#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include "eraser_dropper.hpp"

int main(int argc, char **argv) {
  if (gpioInitialise() < 0) {
    return 1;
  }

  ros::init(argc, argv, "eraser_dropper");
  ros::NodeHandle nh;
  int min_drop_interval, servo_gpio, servo_min_pulse, servo_max_pulse;
  nh.param("min_drop_interval", min_drop_interval, 5);
  nh.param("servo_gpio", servo_gpio, 23);
  nh.param("servo_min_pulse", servo_min_pulse, 1000);
  nh.param("servo_max_pulse", servo_max_pulse, 2000);
  EraserDropper::min_drop_interval = min_drop_interval;
  EraserDropper::servo_gpio = servo_gpio;
  EraserDropper::servo_min_pulse = servo_min_pulse;
  EraserDropper::servo_max_pulse = servo_max_pulse;

  ros::ServiceServer srv = nh.advertiseService<std_srvs::Trigger::Request, std_srvs::Trigger::Response>("drop_eraser", EraserDropper());
  ros::spin();

  return 0;
}
