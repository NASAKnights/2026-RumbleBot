// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Shoot.h"

Shoot::Shoot(Turret *turret, bool Shoot) : m_turret{turret} 
{
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(m_turret);
  isShoot = Shoot;
}

// Called when the command is initially scheduled.
void Shoot::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void Shoot::Execute() 
{
  if (isShoot){
    m_turret->AllowShooting();
  }
  else{
    m_turret->PauseShooting();
  }
}

// Called once the command ends or is interrupted.
void Shoot::End(bool interrupted) 
{
  m_turret->PauseShooting();
}

// Returns true when the command should end.
bool Shoot::IsFinished() {
  if (!isShoot){
    return true;
  }
  return false;
}
