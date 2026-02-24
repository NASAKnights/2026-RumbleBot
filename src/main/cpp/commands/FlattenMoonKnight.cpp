// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/FlattenMoonKnight.h"

FlattenMoonKnight::FlattenMoonKnight(Turret *turret, Wrist *wrist) : m_turret{turret}, m_wrist{wrist}
{

  // Use addRequirements() here to declare subsystem dependencies.

  AddRequirements(m_turret);
  AddRequirements(m_wrist);
}

// Called when the command is initially scheduled.
void FlattenMoonKnight::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void FlattenMoonKnight::Execute() 
{
  m_turret->ChangeHoodAngle(units::angle::radian_t(0.0));
  m_wrist->SetAngle(3.0);
}

// Called once the command ends or is interrupted.
void FlattenMoonKnight::End(bool interrupted) {}

// Returns true when the command should end.
bool FlattenMoonKnight::IsFinished() {
  return false;
}
