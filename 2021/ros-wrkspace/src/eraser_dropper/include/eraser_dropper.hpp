#ifndef __ERASER_DROPPER_HPP__
#define __ERASER_DROPPER_HPP__

#include <ros/ros.h>
#include <std_srvs/Trigger.h>
extern "C" {
  #include <pigpio.h>
}

class EraserDropper {
public:
  bool operator() (std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response) {
    if (ros::Time::now() - previous_drop_time > ros::Duration(min_drop_interval)) {
      gpioServo(servo_gpio, servo_min_pulse);
      ros::Duration(0.5).sleep();
      gpioServo(servo_gpio, servo_max_pulse);
      previous_drop_time = ros::Time::now();
      response.success = true;
      return true;
    }
    response.success = false;
    return false;
  }

  ros::Time previous_drop_time;
  static int min_drop_interval;

  // servo settings
  static int servo_gpio;
  static int servo_min_pulse;
  static int servo_max_pulse;
};

int EraserDropper::min_drop_interval = 5;
int EraserDropper::servo_gpio = -1;
int EraserDropper::servo_min_pulse = -1;
int EraserDropper::servo_max_pulse = -1;

#endif
