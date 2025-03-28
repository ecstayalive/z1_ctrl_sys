#include "arm_controller/js_api.h"

namespace arm_controller {
JsRos::JsRos(const ros::NodeHandle &nh) : nh_(nh) {
  std::string topic_name;
  nh_.param<std::string>("js_topic", topic_name, "/joy");
  js_sub_ = nh_.subscribe(topic_name, 10, &JsRos::callback, this,
                          ros::TransportHints().tcpNoDelay());
}

bool JsRos::connected() const {
  return connected_ and ros::Time::now() - last_recv_time_ < ros::Duration(0.1);
}

void JsRos::getState(js::JsState &js_state) {
  if (!connected_) {
    js_state = {};
    return;
  }

  std::lock_guard<std::mutex> lock(mtx_);
  js_state.A() = mapping_->A();
  js_state.B() = mapping_->B();
  js_state.X() = mapping_->X();
  js_state.Y() = mapping_->Y();
  js_state.LB() = mapping_->LB();
  js_state.RB() = mapping_->RB();
  js_state.Select() = mapping_->Select();
  js_state.Start() = mapping_->Start();
  js_state.LAS() = mapping_->LAS();
  js_state.RAS() = mapping_->RAS();
  js_state.Up() = mapping_->Up();
  js_state.Down() = mapping_->Down();
  js_state.Left() = mapping_->Left();
  js_state.Right() = mapping_->Right();

  js_state.lasX() = mapping_->lasX();
  js_state.lasY() = mapping_->lasY();
  js_state.rasX() = mapping_->rasX();
  js_state.rasY() = mapping_->rasY();
  js_state.lt() = mapping_->lt();
  js_state.rt() = mapping_->rt();
  js_state.LAS() = mapping_->LAS();
  js_state.RAS() = mapping_->RAS();

  for (auto &button : slots_.buttons) {
    button.on_press = button.on_release = false;
  }
  for (auto &button : slots_.axis_buttons_) {
    button.on_press = button.on_release = false;
  }
}

void JsRos::callback(const sensor_msgs::Joy::ConstPtr &msg) {
  std::lock_guard<std::mutex> _(mtx_);
  last_recv_time_ = msg->header.stamp;
  if (not connected_) {
    for (int i{}; i < msg->axes.size(); ++i) {
      axisHandler(i, msg->axes[i]);
    }
    for (int i{}; i < msg->buttons.size(); ++i) {
      buttonHandler(i, msg->buttons[i]);
    }
    connected_ = true;
  } else {
    for (int i{}; i < msg->axes.size(); ++i) {
      if (msg->axes[i] != last_msg_.axes[i]) {
        axisHandler(i, msg->axes[i]);
      }
    }
    for (int i{}; i < msg->buttons.size(); ++i) {
      if (msg->buttons[i] != last_msg_.buttons[i]) {
        buttonHandler(i, msg->buttons[i]);
      }
    }
  }
  last_msg_ = *msg;
}

void JsRos::axisHandler(std::size_t aid, float val) {
  if (aid >= js::JsSlots::NAxes) return;
  slots_.axes[aid] = -val;

  updateAxisButton(2 * aid, val > 0.99);
  updateAxisButton(2 * aid + 1, val < -0.99);
}

void JsRos::updateAxisButton(std::size_t idx, bool pressed) {
  if (pressed) {
    if (not slots_.axis_buttons_[idx].pressed) {
      slots_.axis_buttons_[idx].on_press = true;
    }
    slots_.axis_buttons_[idx].pressed = true;
  } else {
    if (slots_.axis_buttons_[idx].pressed) {
      slots_.axis_buttons_[idx].on_release = true;
    }
    slots_.axis_buttons_[idx].pressed = false;
  }
}

void JsRos::buttonHandler(std::size_t bid, bool val) {
  if (bid >= js::JsSlots::NButtons) return;
  slots_.buttons[bid].pressed = val;
  if (val) {
    slots_.buttons[bid].on_press = true;
  } else {
    slots_.buttons[bid].on_release = true;
  }
}

}  // namespace arm_controller
