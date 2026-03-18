// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/TurretIntake.h"

TurretIntake::TurretIntake()
{
    rev::spark::SparkMaxConfig config;
    config.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kCoast);
    config.SmartCurrentLimit(30);
    m_intakeMotor.Configure(config, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);
}

// This method will be called once per scheduler run
void TurretIntake::Periodic() {}

void TurretIntake::Intake()
{
    // m_intakeMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -0.85);
    m_intakeMotor.Set(-0.85);
}

void TurretIntake::Outtake()
{
    // m_intakeMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.85);
    m_intakeMotor.Set(0.85);
}

void TurretIntake::StopIntake()
{
    // m_intakeMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
    m_intakeMotor.Set(0.0);
}