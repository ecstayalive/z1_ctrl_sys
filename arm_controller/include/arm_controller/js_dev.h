#ifndef JS_DEV_H_
#define JS_DEV_H_
#include <iostream>
namespace arm_controller {
namespace js {

using Axis = float;

struct Button {
  bool on_press{false}, pressed{false}, on_release{false};

  bool operator==(const Button &other) const {
    return on_press == other.on_press && pressed == other.pressed &&
           on_release == other.on_release;
  }

  void update(bool pressed, bool last_pressed) {
    this->pressed = pressed;
    if (pressed and not last_pressed) this->on_press = true;
    if (last_pressed and not pressed) this->on_release = true;
  }

  void clearOnStates() { on_press = on_release = false; }
};

class JsState {
 public:
  enum Type {
    XboxLike,
    Skydroid,
  };

  JsState() { lt() = rt() = -1.; }

  Type &type() { return type_; }
  const Type &type() const { return type_; }

  Button &A() { return buttons_[0]; }
  Button &B() { return buttons_[1]; }
  Button &X() { return buttons_[2]; }
  Button &Y() { return buttons_[3]; }
  Button &LB() { return buttons_[4]; }
  Button &RB() { return buttons_[5]; }
  Button &Select() { return buttons_[6]; }
  Button &Start() { return buttons_[7]; }
  Button &LAS() { return buttons_[8]; }
  Button &RAS() { return buttons_[9]; }
  Button &Up() { return buttons_[10]; }
  Button &Down() { return buttons_[11]; }
  Button &Left() { return buttons_[12]; }
  Button &Right() { return buttons_[13]; }

  const Button &A() const { return buttons_[0]; }
  const Button &B() const { return buttons_[1]; }
  const Button &X() const { return buttons_[2]; }
  const Button &Y() const { return buttons_[3]; }
  const Button &LB() const { return buttons_[4]; }
  const Button &RB() const { return buttons_[5]; }
  const Button &Select() const { return buttons_[6]; }
  const Button &Start() const { return buttons_[7]; }
  const Button &LAS() const { return buttons_[8]; }
  const Button &RAS() const { return buttons_[9]; }
  const Button &Up() const { return buttons_[10]; }
  const Button &Down() const { return buttons_[11]; }
  const Button &Left() const { return buttons_[12]; }
  const Button &Right() const { return buttons_[13]; }

  Axis &lasX() { return axes_[0]; }
  Axis &lasY() { return axes_[1]; }
  Axis &rasX() { return axes_[2]; }
  Axis &rasY() { return axes_[3]; }
  Axis &lt() { return axes_[4]; }
  Axis &rt() { return axes_[5]; }

  Axis lasX() const { return axes_[0]; }
  Axis lasY() const { return axes_[1]; }
  Axis rasX() const { return axes_[2]; }
  Axis rasY() const { return axes_[3]; }
  Axis lt() const { return axes_[4]; }
  Axis rt() const { return axes_[5]; }

  uint8_t &SW1() { return switches_[0]; }
  uint8_t &SW2() { return switches_[1]; }
  uint8_t &SW3() { return switches_[2]; }
  uint8_t &SW4() { return switches_[3]; }

  const uint8_t &SW1() const { return switches_[0]; }
  const uint8_t &SW2() const { return switches_[1]; }
  const uint8_t &SW3() const { return switches_[2]; }
  const uint8_t &SW4() const { return switches_[3]; }

  bool operator==(const JsState &other) const {
    return type_ == other.type_ &&
           std::equal(std::begin(buttons_), std::end(buttons_),
                      std::begin(other.buttons_)) &&
           std::equal(std::begin(axes_), std::end(axes_),
                      std::begin(other.axes_)) &&
           std::equal(std::begin(switches_), std::end(switches_),
                      std::begin(other.switches_));
  }

 private:
  Type type_{XboxLike};
  Button buttons_[14]{};
  Axis axes_[6]{};
  uint8_t switches_[4]{};
};

struct JsSlots {
  static constexpr std::size_t NAxes = 8;
  static constexpr std::size_t NButtons = 16;

  Axis axes[NAxes]{};
  Button axis_buttons_[2 * NAxes]{};
  Button buttons[NButtons]{};
};

struct JsMapping {
  virtual ~JsMapping() = default;
  explicit JsMapping(JsSlots &slots) : slots_(slots) {}
  virtual Button A() const { return {}; }
  virtual Button B() const { return {}; }
  virtual Button X() const { return {}; }
  virtual Button Y() const { return {}; }
  virtual Button LB() const { return {}; }
  virtual Button RB() const { return {}; }
  virtual Button Select() const { return {}; }
  virtual Button Start() const { return {}; }
  virtual Button LAS() const { return {}; }
  virtual Button RAS() const { return {}; }
  virtual Button Up() const { return {}; }
  virtual Button Down() const { return {}; }
  virtual Button Left() const { return {}; }
  virtual Button Right() const { return {}; }

  virtual Axis lasX() const { return {}; }
  virtual Axis lasY() const { return {}; }
  virtual Axis rasX() const { return {}; }
  virtual Axis rasY() const { return {}; }
  virtual Axis lt() const { return {}; }
  virtual Axis rt() const { return {}; }

 protected:
  JsSlots &slots_;
};

struct BeitongMapping : JsMapping {
  explicit BeitongMapping(JsSlots &slots) : JsMapping(slots) {
    slots_.axes[5] = slots_.axes[4] = -1.;
  }

  Button A() const override { return slots_.buttons[0]; }
  Button B() const override { return slots_.buttons[1]; }
  Button X() const override { return slots_.buttons[3]; }
  Button Y() const override { return slots_.buttons[4]; }
  Button LB() const override { return slots_.buttons[6]; }
  Button RB() const override { return slots_.buttons[7]; }
  Button Select() const override { return slots_.buttons[10]; }
  Button Start() const override { return slots_.buttons[11]; }
  Button LAS() const override { return slots_.buttons[13]; }
  Button RAS() const override { return slots_.buttons[14]; }
  Button Up() const override { return slots_.axis_buttons_[14]; }
  Button Down() const override { return slots_.axis_buttons_[15]; }
  Button Left() const override { return slots_.axis_buttons_[12]; }
  Button Right() const override { return slots_.axis_buttons_[13]; }

  Axis lasX() const override { return slots_.axes[0]; }
  Axis lasY() const override { return slots_.axes[1]; }
  Axis rasX() const override { return slots_.axes[2]; }
  Axis rasY() const override { return slots_.axes[3]; }
  Axis lt() const override { return slots_.axes[5]; }
  Axis rt() const override { return slots_.axes[4]; }
};

struct XboxMapping : JsMapping {
  explicit XboxMapping(JsSlots &slots) : JsMapping(slots) {
    slots_.axes[2] = slots_.axes[5] = -1.;
  }

  Button A() const override { return slots_.buttons[0]; }
  Button B() const override { return slots_.buttons[1]; }
  Button X() const override { return slots_.buttons[2]; }
  Button Y() const override { return slots_.buttons[3]; }
  Button LB() const override { return slots_.buttons[4]; }
  Button RB() const override { return slots_.buttons[5]; }
  Button Select() const override { return slots_.buttons[6]; }
  Button Start() const override { return slots_.buttons[7]; }
  Button LAS() const override { return slots_.buttons[9]; }
  Button RAS() const override { return slots_.buttons[10]; }
  Button Up() const override { return slots_.axis_buttons_[14]; }
  Button Down() const override { return slots_.axis_buttons_[15]; }
  Button Left() const override { return slots_.axis_buttons_[12]; }
  Button Right() const override { return slots_.axis_buttons_[13]; }

  Axis lasX() const override { return slots_.axes[0]; }
  Axis lasY() const override { return slots_.axes[1]; }
  Axis rasX() const override { return slots_.axes[3]; }
  Axis rasY() const override { return slots_.axes[4]; }
  Axis lt() const override { return slots_.axes[2]; }
  Axis rt() const override { return slots_.axes[5]; }
};

}  // namespace js
}  // namespace arm_controller

#endif  // LQC_JS_DEV_H_
