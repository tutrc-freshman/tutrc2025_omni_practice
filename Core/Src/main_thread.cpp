#include <stm32rcos/core.hpp>

extern "C" void main_thread(void *) {
  while (true) {
    osDelay(1);
  }
}