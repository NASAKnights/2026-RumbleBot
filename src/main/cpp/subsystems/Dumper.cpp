// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Dumper.h"

Dumper::Dumper()
{

}

void Dumper::SetRotationSpeed(units::revolutions_per_minute_t speed)
{
    auto motorRequest = ctre::phoenix6::controls::VelocityVoltage{speed};
    //set leader to angular velocity
    ctre::phoenix::StatusCode TopLeftStatus = m_TopLeftMotor.SetControl(motorRequest.WithVelocity(speed).WithSlot(0));
    if (TopLeftStatus != ctre::phoenix::StatusCode::OK)
    {
        frc::SmartDashboard::PutNumber("Dumper/SetRotationSpeed/StatusCode", static_cast<int>(TopLeftStatus));
    }
    

}

// This method will be called once per scheduler run
void Dumper::Periodic() {}
