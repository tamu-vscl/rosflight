#ifndef DISTURBANCE_INJECTOR_H
#define DISTURBANCE_INJECTOR_H

#define SIGNAL_TYPE_OFFSET 0
#define SIGNAL_TYPE_SQUARE 1
#define SIGNAL_TYPE_RAMP 2
#define SIGNAL_TYPE_SINE 3

#define PI 3.14159

#include <ros/ros.h>

#include <rosflight_msgs/DisturbInput.h>
#include <rosflight_msgs/OffboardOverride.h>
#include <rosflight_msgs/ParamGet.h>
#include <rosflight_msgs/RCRaw.h>

class DisturbanceInjector
{
public:

  struct Signal
  {
    int type = SIGNAL_TYPE_OFFSET;
    double period = 1;
    double amplitude = 0;
    double offset = 0;
    double last_update = -1;
    double percent_of_period = 0;
    double period_count = 0;
    double length = 1;
    double value = 0;
    double settling_time = 5;
    double update_rate = 100;
  };

  DisturbanceInjector();
  inline const Signal& signal() const { return signal_; }

private:

  ros::NodeHandle nh_;
  ros::Publisher offboard_override_pub_;
  ros::Subscriber rc_raw_sub_;
  ros::ServiceServer arm_disturb_input_svc_;
  ros::ServiceServer arm_disturb_all_inputs_svc_;
  ros::ServiceClient param_get_client_;

  bool arm_disturb_input_callback(rosflight_msgs::DisturbInput::Request  &req,
                                  rosflight_msgs::DisturbInput::Response &res);
  bool arm_disturb_all_inputs_callback(rosflight_msgs::DisturbInput::Request  &req,
                                       rosflight_msgs::DisturbInput::Response &res);
  void update_signal();
  void load_param();

  Signal signal_;
  double arm_time_out_time_;
};

#endif // DISTURBANCE_INJECTOR_H
