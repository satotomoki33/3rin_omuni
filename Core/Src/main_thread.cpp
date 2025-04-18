#include <cmath>
#include <cstdint>
#include <cstdio>
#include <mutex>

#include <stm32rcos/core.hpp>
#include <stm32rcos/hal.hpp>
#include <stm32rcos/module/bno055.hpp>
#include <stm32rcos/module/c6x0.hpp>
#include <stm32rcos/module/ps3.hpp>
#include <stm32rcos/peripheral/bxcan.hpp>

#include "gyro.hpp"
#include "omuni3.hpp"
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim8;

stm32rcos::core::Mutex *imu_quat_mtx;
Eigen::Quaternionf imu_quat;

extern "C" void main_thread(void *) {
  using namespace stm32rcos::core;
  using namespace stm32rcos::peripheral;
  using namespace stm32rcos::module;

  imu_quat_mtx = new Mutex(); // bno用

  UART uart2(&huart2);
  uart2.enable_stdout();
  UART uart5(&huart5);
  PS3 ps3(uart5);

  UART uart4(&huart4);
  BNO055 bno(uart4);
  bno.start(500);

  Thread com_thread1(
      [](void *args) {
        auto bno = reinterpret_cast<BNO055 *>(args);
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

  BxCAN can1(&hcan1);
  C6x0Manager c610_manager(can1);
  C6x0 c610_1(c610_manager, C6x0Type::C610, C6x0ID::_1);
  C6x0 c610_2(c610_manager, C6x0Type::C610, C6x0ID::_2);
  C6x0 c610_3(c610_manager, C6x0Type::C610, C6x0ID::_3);
  can1.start();

  omuni3 omuni3;
  Tire tire_actual;
  Tire tire_output;

  float target_theta = 0;

  while (true) {
    std::uint32_t start = osKernelGetTickCount();
    float theta;
    {
      std::lock_guard lock(*imu_quat_mtx);
      theta = zero_to_2pi(quaternion_to_yaw(imu_quat));
    }
    ps3.update();
    c610_manager.update();

    control.x = ps3.get_axis(PS3Axis::RIGHT_X);
    control.y = ps3.get_axis(PS3Axis::RIGHT_Y);
    // control.turnspeed = 50 * ps3.get_axis(PS3Axis::LEFT_X);
    if (ps3.get_key_down(PS3Key::UP)) {
      target_theta = 0;
    } else if (ps3.get_key_down(PS3Key::DOWN)) {
      target_theta = M_PI;
    } else if (ps3.get_key_down(PS3Key::LEFT)) {
      target_theta = 3 * M_PI / 2;
    } else if (ps3.get_key_down(PS3Key::RIGHT)) {
      target_theta = M_PI / 2;
    }
    double theta_error = target_theta - theta;
    if (theta_error > M_PI) {
      target_theta -= 2 * M_PI;
    } else if (theta_error < -M_PI) {
      target_theta += 2 * M_PI;
    } 

    control.turnspeed = 80 * theta_error;

    tire_actual.Tire_1 = c610_1.get_rps();
    tire_actual.Tire_2 = c610_2.get_rps();
    tire_actual.Tire_3 = c610_3.get_rps();
    omuni3.get_speed(tire_actual);
    tire_output = omuni3.output(control, 300, theta);

    c610_1.set_current(tire_output.Tire_1);
    c610_2.set_current(tire_output.Tire_2);
    c610_3.set_current(tire_output.Tire_3);

    c610_manager.transmit();
    printf("theta: %f, target_theta: %f, control.x: %f, control.y: %f, "
           "control.turnspeed: %f\r\n",
           theta, target_theta, control.x, control.y, control.turnspeed);
    osDelayUntil(start + 10);
  }
}