// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <units/length.h>
#include <frc/AnalogInput.h>

class Sharp_IRDistanceSensor {
 public:
  Sharp_IRDistanceSensor(int channel);
  units::centimeter_t GetDistance();

 private:
  frc::AnalogInput m_input;
};
