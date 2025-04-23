#include <cmath>
#include <cstdint>
#include <cstdio>
#include <mutex>

#include <stm32rcos/core.hpp>
#include <stm32rcos/hal.hpp>
#include <stm32rcos/peripheral.hpp>

#include <stm32rcos_drivers/bno055.hpp>
#include <stm32rcos_drivers/c6x0.hpp>
#include <stm32rcos_drivers/ps3.hpp>

#include "gyro.hpp"
#include "omuni3.hpp"

using stm32rcos::core::Mutex;
using stm32rcos::core::Thread;
using stm32rcos::peripheral::Can;
using stm32rcos::peripheral::Uart;

using namespace stm32rcos_drivers;

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim8;

Mutex *imu_quat_mtx;
Eigen::Quaternionf imu_quat;

extern "C" void main_thread(void *) {
  imu_quat_mtx = new Mutex(); // bno用

  Uart uart2(&huart2);
  enable_stdout(uart2);
  Uart uart5(&huart5);
  Ps3 ps3(uart5);

  Uart uart4(&huart4);
  Bno055 bno(uart4);
  bno.start(500);

  Thread com_thread1(
      [](void *args) {
        auto bno = reinterpret_cast<Bno055 *>(args);
        while (true) {
          uint32_t start_time = osKernelGetTickCount();
          auto quat = bno->get_quaternion();
          if (quat) {
            std::lock_guard lock(*imu_quat_mtx);
            imu_quat = quat.value();
          }
          osDelayUntil(start_time + 10);
        }
      },
      &bno, 4096, osPriorityNormal);

  Control control;

  Can can1(&hcan1);
  C6x0Manager c610_manager(can1);
  C6x0 c610_1(c610_manager, C6x0Type::C610, C6x0Id::ID_1);
  C6x0 c610_2(c610_manager, C6x0Type::C610, C6x0Id::ID_2);
  C6x0 c610_3(c610_manager, C6x0Type::C610, C6x0Id::ID_3);
  can1.start();

  float target_theta = 0;

  while (true) {
    uint32_t start = osKernelGetTickCount();
    float theta;
    {
      std::lock_guard lock(*imu_quat_mtx);
      theta = zero_to_2pi(quaternion_to_yaw(imu_quat));
    }
    ps3.update();
    c610_manager.update();

    control.x = ps3.get_axis(Ps3Axis::RIGHT_X);
    control.y = ps3.get_axis(Ps3Axis::RIGHT_Y);
    // control.turnspeed = 50 * ps3.get_axis(Ps3Axis::LEFT_X);
    if (ps3.get_key_down(Ps3Key::UP)) {
      target_theta = 0;
    } else if (ps3.get_key_down(Ps3Key::DOWN)) {
      target_theta = M_PI;
    } else if (ps3.get_key_down(Ps3Key::LEFT)) {
      target_theta = 3 * M_PI / 2;
    } else if (ps3.get_key_down(Ps3Key::RIGHT)) {
      target_theta = M_PI / 2;
    }
    double theta_error = target_theta - theta;
    if (theta_error > M_PI) {
      target_theta -= 2 * M_PI;
    } else if (theta_error < -M_PI) {
      target_theta += 2 * M_PI;
    }

    control.turnspeed = 80 * theta_error;

    Tire tire_actual;
    tire_actual.Tire_1 = c610_1.get_rpm() / 60.0f;
    tire_actual.Tire_2 = c610_2.get_rpm() / 60.0f;
    tire_actual.Tire_3 = c610_3.get_rpm() / 60.0f;
    Tire tire_output = omni_output(100, tire_actual, control, 300, theta);

    c610_1.set_current_ref(tire_output.Tire_1);
    c610_2.set_current_ref(tire_output.Tire_2);
    c610_3.set_current_ref(tire_output.Tire_3);

    c610_manager.transmit();
    printf("theta: %f, target_theta: %f, control.x: %f, control.y: %f, "
           "control.turnspeed: %f\r\n",
           theta, target_theta, control.x, control.y, control.turnspeed);
    osDelayUntil(start + 10);
  }
}