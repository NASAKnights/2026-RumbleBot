// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/HalfRaiseIntake.h"

HalfRaiseIntake::HalfRaiseIntake(TurretIntake *_turretIntake, Wrist *_wrist, double _angle) : 
m_turretIntake{_turretIntake}, m_wrist{_wrist}
{
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(m_turretIntake);
  AddRequirements(m_wrist);
  m_angle = _angle;
}

// Called when the command is initially scheduled.
void HalfRaiseIntake::Initialize() 
{
  m_wrist->SetAngle(m_angle);
}

// Called repeatedly when this Command is scheduled to run
void HalfRaiseIntake::Execute() {
  m_turretIntake->Intake();
}

// Called once the command ends or is interrupted.
void HalfRaiseIntake::End(bool interrupted) {
  m_wrist->SetAngle(3.0);
  m_turretIntake->StopIntake();
}

// Returns true when the command should end.
bool HalfRaiseIntake::IsFinished() {
  return false;
}
