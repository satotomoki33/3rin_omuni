#include <cmath>
#include <cstdint>
#include <cstdio>

#include <stm32rcos/core.hpp>
#include <stm32rcos/hal.hpp>
#include <stm32rcos/module/c6x0.hpp>
#include <stm32rcos/module/ps3.hpp>
#include <stm32rcos/peripheral/bxcan.hpp>

#include "omuni3.hpp"
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim8;

extern "C" void main_thread(void *) {
  using namespace stm32rcos::core;
  using namespace stm32rcos::peripheral;
  using namespace stm32rcos::module;

  UART uart2(&huart2);
  uart2.enable_stdout();
  UART uart5(&huart5);
  PS3 ps3(uart5);
  Control control;

  BxCAN can1(&hcan1);
  C6x0Manager c610_manager(can1);
  C6x0 c610_1(c610_manager, C6x0Type::C610, C6x0ID::_1);
  C6x0 c610_2(c610_manager, C6x0Type::C610, C6x0ID::_2);
  C6x0 c610_3(c610_manager, C6x0Type::C610, C6x0ID::_3);
  can1.start();

  omuni3 omuni3;
  Tire tire_actual;
  Tire tire_output;

  while (true) {
    std::uint32_t start = osKernelGetTickCount();
    ps3.update();
    c610_manager.update();

    control.x = ps3.get_axis(PS3Axis::RIGHT_X);
    control.y = ps3.get_axis(PS3Axis::RIGHT_Y);
    control.turnspeed = 50 * ps3.get_axis(PS3Axis::LEFT_X);

    tire_actual.Tire_1 = c610_1.get_rps();
    tire_actual.Tire_2 = c610_2.get_rps();
    tire_actual.Tire_3 = c610_3.get_rps();
    omuni3.get_speed(tire_actual);
    tire_output = omuni3.output(control, 130);

    c610_1.set_current(tire_output.Tire_1);
    c610_2.set_current(tire_output.Tire_2);
    c610_3.set_current(tire_output.Tire_3);

    c610_manager.transmit();
    osDelayUntil(start + 10);
  }
}