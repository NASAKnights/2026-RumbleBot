// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Intake.h"

Intake::Intake(TurretIntake *_turretIntake, Wrist *_wrist) : m_turretIntake{_turretIntake}, m_wrist{_wrist} 
{
  // Use addRequirements() here to declare subsystem dependencies.

  AddRequirements(m_turretIntake);
  AddRequirements(m_wrist);
}

// Called when the command is initially scheduled.
void Intake::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void Intake::Execute() 
{
  m_turretIntake->Intake();
  m_wrist->SetAngle(3.0);
}

// Called once the command ends or is interrupted.
void Intake::End(bool interrupted) {
  m_turretIntake->StopIntake();
}

// Returns true when the command should end.
bool Intake::IsFinished() {
  return false;
}
