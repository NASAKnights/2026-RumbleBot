// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Turret_Shooter.h"

Turret_Shooter::Turret_Shooter()
{
    ctre::phoenix6::configs::TalonFXSConfiguration leftMotorConfig{};
    ctre::phoenix6::configs::TalonFXSConfiguration rightMotorConfig{};
    // ctre::phoenix6::controls::Follower LeftFollower{m_rightMotor.GetDeviceID(), true};
    leftMotorConfig.Commutation.WithMotorArrangement(ctre::phoenix6::signals::MotorArrangementValue::Minion_JST);
    rightMotorConfig.Commutation.WithMotorArrangement(ctre::phoenix6::signals::MotorArrangementValue::Minion_JST);
    // m_leftMotor.SetControl(LeftFollower);
    ctre::phoenix6::configs::Slot0Configs motorSlot0Configs{};
    motorSlot0Configs.kP = kP;
    motorSlot0Configs.kI = kI;
    motorSlot0Configs.kD = kD;
    motorSlot0Configs.kS = kS;
    motorSlot0Configs.kA = kA;
    motorSlot0Configs.kV = kV;
    leftMotorConfig.Slot0 = motorSlot0Configs;
    rightMotorConfig.Slot0 = motorSlot0Configs;
    m_leftMotor.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Coast);
    m_rightMotor.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Coast);
    ctre::phoenix6::configs::CurrentLimitsConfigs currentConfig{};
    currentConfig.SupplyCurrentLimitEnable = kEnableCurrentLimit;
    currentConfig.SupplyCurrentLimit = kPeakCurrentLimit;
    currentConfig.SupplyCurrentLowerLimit = kContinousCurrentLimit;
    currentConfig.SupplyCurrentLowerTime = kPeakCurrentDuration;
    leftMotorConfig.CurrentLimits = currentConfig;
    rightMotorConfig.CurrentLimits = currentConfig;

    leftMotorConfig.MotorOutput.Inverted = true;
    rightMotorConfig.MotorOutput.Inverted = false;

    ctre::phoenix::StatusCode leftStatus = m_leftMotor.GetConfigurator().Apply(leftMotorConfig);
    ctre::phoenix::StatusCode rightStatus = m_rightMotor.GetConfigurator().Apply(rightMotorConfig);

    frc::SmartDashboard::PutBoolean("/Turret/Shooter/Left Motor Status", false);
    frc::SmartDashboard::PutBoolean("/Turret/Shooter/Right Motor Status", false);

    // while (!leftStatus.IsOK()) {
    // ctre::phoenix::StatusCode leftStatus = m_leftMotor.GetConfigurator().Apply(leftMotorConfig);
    // }
    // while (!rightStatus.IsOK()) {
    // ctre::phoenix::StatusCode rightStatus = m_rightMotor.GetConfigurator().Apply(rightMotorConfig);
    // }

    frc::SmartDashboard::PutBoolean("/Turret/Shooter/Left Motor Status", leftStatus.IsOK());
    frc::SmartDashboard::PutBoolean("/Turret/Shooter/Right Motor Status", rightStatus.IsOK());

    frc::SmartDashboard::PutNumber("/Turret/Shooter/Commanded Ball Speed MPS", 0.0);
    frc::SmartDashboard::PutNumber("/Turret/Shooter/Commanded Motor RPM", 0.0);

    frc::SmartDashboard::PutBoolean("/Turret/Shooter/Ball Speed Manual Override", false);
    frc::SmartDashboard::PutNumber("/Turret/Shooter/Ball Speed Manual Set MPS", 0.0);

    frc::SmartDashboard::PutNumber("/Turret/Shooter/Actual Motor RPM", 0.0);
    frc::SmartDashboard::PutNumber("/Turret/Shooter/Actual Ball Speed MPS", 0.0);

    frc::SmartDashboard::PutNumber("/Turret/Shooter/Left Motor Voltage", 0.0);
    frc::SmartDashboard::PutNumber("/Turret/Shooter/Right Motor Voltage", 0.0);

    
    frc::SmartDashboard::PutBoolean("/Turret/Spindexer Indexer/Running", false);
}

void Turret_Shooter::SetSpeed(units::meters_per_second_t ballSpeed) {
    units::turns_per_second_t motorSpeed = (kFlyWheelVelocityGain * ballSpeed * units::radian_t{1} * 4.0) / (kFlywheelDiameter * kGearRatio);

    //Spin motor 10% faster than needed to account for loss of speed when shooting rapidly
    motorSpeed += motorSpeed * Turret_ShooterConstants::kPercentBoost;
    
    auto motorRequest = ctre::phoenix6::controls::VelocityVoltage{motorSpeed};
    ctre::phoenix::StatusCode leftStatus = m_leftMotor.SetControl(motorRequest.WithVelocity(motorSpeed).WithSlot(0));
    ctre::phoenix::StatusCode rightStatus = m_rightMotor.SetControl(motorRequest.WithVelocity(motorSpeed).WithSlot(0));
    frc::SmartDashboard::PutBoolean("/Turret/Shooter/Left Motor Status", leftStatus.IsOK());
    frc::SmartDashboard::PutBoolean("/Turret/Shooter/Right Motor Status", rightStatus.IsOK());
    frc::SmartDashboard::PutNumber("/Turret/Shooter/Commanded Ball Speed MPS", ballSpeed.value());
    frc::SmartDashboard::PutNumber("/Turret/Shooter/Commanded Motor RPM", motorSpeed.value() * 60.0);
}

void Turret_Shooter::RunSpindexerIndexer() 
{
    m_indexerMotor.Set(Turret_ShooterConstants::indexerSpeed);
    m_spindexerMotor.Set(Turret_ShooterConstants::spindexerSpeed);

    frc::SmartDashboard::PutBoolean("/Turret/Spindexer Indexer/Running", true);
 }

void Turret_Shooter::StopSpindexerIndexer()
{
    m_indexerMotor.Set(0.0);
    m_spindexerMotor.Set(0.0);

    frc::SmartDashboard::PutBoolean("/Turret/Spindexer Indexer/Running", false);
}

void Turret_Shooter::StopMotors()
{
    auto motorRequest = ctre::phoenix6::controls::VelocityVoltage{0_tps};
    auto motorSpeed = units::radians_per_second_t{0.0};
    m_leftMotor.SetControl(motorRequest.WithVelocity(motorSpeed));
    m_rightMotor.SetControl(motorRequest.WithVelocity(motorSpeed));
    frc::SmartDashboard::PutNumber("/Turret/Shooter/Commanded Ball Speed MPS", 0.0);
    frc::SmartDashboard::PutNumber("/Turret/Shooter/Commanded Motor RPM", 0.0);
}

void Turret_Shooter::RunAll()
{
    SetSpeed(units::meters_per_second_t{18.0});
    m_indexerMotor.Set(-0.5);
    m_spindexerMotor.Set(0.5);
}

void Turret_Shooter::StopAll()
{
    SetSpeed(units::meters_per_second_t{0.0});
    m_indexerMotor.Set(0.0);
    m_spindexerMotor.Set(0.0);
}

units::meters_per_second_t Turret_Shooter::GetActualBallSpeed()
{
    units::turns_per_second_t motorSpeed = m_leftMotor.GetVelocity().GetValue();
    units::meters_per_second_t ballSpeed = motorSpeed * (kFlywheelDiameter * kGearRatio) / (kFlyWheelVelocityGain * units::radian_t{1} * 4.0); 
    return ballSpeed;
}

void Turret_Shooter::Periodic()
{
    frc::SmartDashboard::PutNumber("/Turret/Shooter/Left Motor Voltage", m_leftMotor.GetMotorVoltage().GetValue().value());
    frc::SmartDashboard::PutNumber("/Turret/Shooter/Right Motor Voltage", m_rightMotor.GetMotorVoltage().GetValue().value());

    // left and right motors are same speed
    units::turns_per_second_t motorSpeed = m_leftMotor.GetVelocity().GetValue();
    units::meters_per_second_t ballSpeed = motorSpeed * (kFlywheelDiameter * kGearRatio) / (kFlyWheelVelocityGain * units::radian_t{1} * 4.0); 

    frc::SmartDashboard::PutNumber("/Turret/Shooter/Actual Motor RPM", motorSpeed.value() * 60.0);
    frc::SmartDashboard::PutNumber("/Turret/Shooter/Actual Ball Speed MPS", ballSpeed.value()); 

    bool override = frc::SmartDashboard::GetBoolean("/Turret/Shooter/Ball Speed Manual Override", false);
    if (override) {
        SetSpeed(units::meters_per_second_t{frc::SmartDashboard::GetNumber("/Turret/Shooter/Ball Speed Manual Set MPS", 0.0)});
    }
}
