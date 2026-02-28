// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.ßöä
#include "subsystems/Climber.h"

Climber::Climber() : 
    climberFollower(climberMotor1.GetDeviceID(), false)
{

    // climberMotor2.SetControl(climberFollower);

    // Initialize Climber Logging
    wpi::log::DataLog& log = frc::DataLogManager::GetLog();
    m_PositionLog = wpi::log::DoubleLogEntry(log, "/Climber/Position");
    m_StateLog = wpi::log::IntegerLogEntry(log, "/Climber/State");
    m_LimitSwitchLog = wpi::log::BooleanLogEntry(log, "/Climber/LimitSwitch");
}

// This method will be called once per scheduler run
void Climber::Periodic() {
  frc::SmartDashboard::PutBoolean("Climber at Bot?",bottomLimit1.Get());
  frc::SmartDashboard::PutNumber("Climber_Position",climberMotor1.GetPosition().GetValueAsDouble());

  // Write out to Log file
  m_PositionLog.Append(climberMotor1.GetPosition().GetValueAsDouble());
  m_StateLog.Append(m_ClimberState);
  m_LimitSwitchLog.Append(bottomLimit1.Get());

}

void Climber::moveMotor() {
    climberMotor1.Set(0.1); // retracts when set to 0.1
}

void Climber::stopMotor() {
    climberMotor1.Set(0.0);
}

void Climber::Zero() {
    if (!bottomLimit1.Get())
    {
        climberMotor1.Set(-0.1);
    }
    else 
    {
        climberMotor1.Set(0.0);
        //set encoder to zero
    }

}

void Climber::extend() {

}

void Climber::retract(){
  if (climberMotor1.Get())
  {
    /* code */
  }
  
}

// void Climber::retractLimit_Pit(){
    
//     if (bottomLimit1.Get()) {
//       climberMotor1.Set(0.1);
//     }
//     else {
//       //climberMotor1.StopMotor();
//       climberMotor1.Set(0);
//       while(climberMotor1.SetPosition(units::angle::turn_t{0}) != ctre::phoenix::StatusCode::OK){};
//     }
// }

// bool Climber::atBottomlimit() {
//   return (!bottomLimit1.Get());
// }