// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Shoot.h"

Shoot::Shoot(Turret *turret) : m_turret{turret} 
{
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(m_turret);
}

// Called when the command is initially scheduled.
void Shoot::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void Shoot::Execute() 
{
  m_turret->AllowShooting();
}

// Called once the command ends or is interrupted.
void Shoot::End(bool interrupted) 
{
  m_turret->PauseShooting();
}

// Returns true when the command should end.
bool Shoot::IsFinished() {
  return false;
}
