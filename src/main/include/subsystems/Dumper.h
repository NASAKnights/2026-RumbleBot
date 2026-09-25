// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <units/velocity.h>
#include <units/angular_velocity.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/smartdashboard/SmartDashboard.h>

namespace DumperConstants {
  //4 kraken x60s
  static const int MotorIDTopLeft = 3;
  static const int MotorIDTopRight = 4;
  static const int MotorIDBottomLeft = 5;
  static const int MotorIDBottomRight = 6;
  
}

class Dumper : public frc2::SubsystemBase {
 public:
  Dumper();

  void Periodic() override;
  void StopMotors();
  void SetSurfaceSpeed(units::velocity::meters_per_second_t speed);
  void SetRotationSpeed(units::revolutions_per_minute_t speed);

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  //one motor leader, three followers
  ctre::phoenix6::hardware::TalonFX m_TopLeftMotor{DumperConstants::MotorIDTopLeft};
  ctre::phoenix6::hardware::TalonFX m_TopRightMotor{DumperConstants::MotorIDTopRight};
  ctre::phoenix6::hardware::TalonFX m_BottomLeftMotor{DumperConstants::MotorIDBottomLeft};
  ctre::phoenix6::hardware::TalonFX m_BottomRightMotor{DumperConstants::MotorIDBottomRight};
};
