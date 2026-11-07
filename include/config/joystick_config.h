#pragma once

namespace robot::joystick_cfg {

// Raw ADC range of the transmitter joysticks.
constexpr int kAdcMin = 0;
constexpr int kAdcMax = 4095;
constexpr int kDeadzone = 100;

// Left stick (forward/backward).
constexpr int kYCenter = 1880;
constexpr int kYSign = -1;

// Right stick (strafe).
constexpr int kXCenter = 1890;
constexpr int kXSign = -1;

// Right stick twist/pot (rotation).
constexpr int kRotateCenter = 1880;
constexpr int kRotateSign = -1;

}  // namespace robot::joystick_cfg
