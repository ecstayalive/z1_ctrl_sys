#ifndef JS_API_H_
#define JS_API_H_

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

#include "js_dev.h"

namespace arm_controller {
class JsRos {
 public:
  template <class Mapping>
  static std::unique_ptr<JsRos> make(const ros::NodeHandle &nh) {
    auto device = std::unique_ptr<JsRos>(new JsRos(nh));
    device->mapping_ = std::make_unique<Mapping>(device->slots_);
    return device;
  }

  bool connected() const;
  void getState(js::JsState &js_state);

 private:
  explicit JsRos(const ros::NodeHandle &nh);

  void callback(const sensor_msgs::Joy::ConstPtr &msg);
  void axisHandler(std::size_t aid, float val);
  void updateAxisButton(std::size_t idx, bool pressed);
  void buttonHandler(std::size_t bid, bool val);

  ros::NodeHandle nh_;
  ros::Subscriber js_sub_;
  ros::Time last_recv_time_;
  std::atomic<bool> connected_{false};
  std::mutex mtx_;
  js::JsSlots slots_;
  std::unique_ptr<js::JsMapping> mapping_;
  sensor_msgs::Joy last_msg_;
};

}  // namespace arm_controller

#endif
