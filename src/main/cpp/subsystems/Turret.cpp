// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#include "subsystems/Turret.h"

#include <frc/geometry/Transform3d.h>
#include <frc/geometry/Translation2d.h>
#include <cmath>
#include <networktables/NetworkTableInstance.h>
#include <algorithm>

namespace
{

    units::turns_per_second_t GetShooterSpeedCorrection(units::degree_t turretAngle, units::turns_per_second_t amplitude)
    {
        // +peak at 90 deg (topspin), -peak at 270 deg (backspin).
        return amplitude * std::sin(units::radian_t{turretAngle}.value());
    }

    units::degree_t GetRobotVelocityTurretAngleCorrection(frc::Pose2d robotPose, units::degree_t KVTurretAngleCompensation, units::meter_t target_distance)
    {

       

        return KVTurretAngleCompensation;
    }

}

// using State = frc::TrapezoidProfile<units::degrees>::State;
using degrees_per_second_squared_t =
    units::unit_t<units::compound_unit<units::angular_velocity::degrees_per_second,
                                       units::inverse<units::time::seconds>>>;

Turret::Turret() : m_controller(
                       TurretConstants::kAngleP, TurretConstants::kAngleI, TurretConstants::kAngleD),
                   // m_motor(TurretConstants::kAngleMotorId, rev::spark::SparkLowLevel::MotorType::kBrushless),
                   m_feedforward(TurretConstants::kFFks, TurretConstants::kFFkg, TurretConstants::kFFkV, TurretConstants::kFFkA)
{
    // m_motor.SetInverted(true);
    rev::spark::SparkBaseConfig config;
    config.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kCoast);
    config.encoder.PositionConversionFactor(TurretConstants::turretPositionConversionFactor);
    config.encoder.VelocityConversionFactor(TurretConstants::turretVelocityConversionFactor);
    config.SmartCurrentLimit(30, 0, 20000);

    m_hood.SetBounds(units::microsecond_t{2000}, units::microsecond_t{1550}, units::microsecond_t{1500}, units::microsecond_t{1450}, units::microsecond_t{1000});
    m_hood2.SetBounds(units::microsecond_t{2000}, units::microsecond_t{1550}, units::microsecond_t{1500}, units::microsecond_t{1450}, units::microsecond_t{1000});

    m_controller.SetIZone(TurretConstants::kIZone);
    m_controller.SetTolerance(TurretConstants::kTolerancePos.value(), TurretConstants::kToleranceVel.value());
    wpi::log::DataLog &log = frc::DataLogManager::GetLog();
    m_AngleLog = wpi::log::DoubleLogEntry(log, "/Turret/Angle");
    m_SetPointLog = wpi::log::DoubleLogEntry(log, "/Turret/Setpoint");
    m_StateLog = wpi::log::IntegerLogEntry(log, "/Turret/State");
    m_MotorCurrentLog = wpi::log::DoubleLogEntry(log, "/Turret/MotorCurrent");
    m_MotorVoltageLog = wpi::log::DoubleLogEntry(log, "/Turret/MotorVoltage");
    m_PoseStaleLog = wpi::log::BooleanLogEntry(log, "/Turret/PoseStale");

    frc::SmartDashboard::PutBoolean("/Turret/Shooter/Allow Shooting", false);

    frc::SmartDashboard::PutBoolean("/Turret/Hood/Angle Manual Override", false);
    frc::SmartDashboard::PutNumber("/Turret/Hood/Angle Manual Set", 0.0);
    frc::SmartDashboard::SetDefaultNumber("/Turret/Comp/TurretAngleAmpDeg", 4.0);
    frc::SmartDashboard::SetDefaultNumber("/Turret/Comp/ShooterSpeedAmpRPS", 0.0);
    frc::SmartDashboard::SetPersistent("/Turret/Comp/TurretAngleAmpDeg");
    frc::SmartDashboard::SetPersistent("/Turret/Comp/ShooterSpeedAmpRPS");
    frc::SmartDashboard::PutNumber(
        "/Turret/Comp/TurretAngleAmpDeg",
        frc::SmartDashboard::GetNumber("/Turret/Comp/TurretAngleAmpDeg", 4.0));
    frc::SmartDashboard::PutNumber(
        "/Turret/Comp/ShooterSpeedAmpRPS",
        frc::SmartDashboard::GetNumber("/Turret/Comp/ShooterSpeedAmpRPS", 0.0));
    networkTableInst = nt::NetworkTableInstance::GetDefault();
    auto poseTable = networkTableInst.GetTable("ROS2Bridge");
    baseLinkSubscriber = poseTable->GetDoubleArrayTopic(robotPoseLink).Subscribe({}, {.periodic = 0.02, .sendAll = true});

    // Initialize goal topic - publish default and subscribe for updates
    auto turretTable = networkTableInst.GetTable("Turret");
    goalPublisher = turretTable->GetDoubleArrayTopic("goal").Publish({.periodic = 0.01, .sendAll = true});
    std::vector<double> defaultGoal = {4.5, 4.0, 1.829};
    goalSubscriber = turretTable->GetDoubleArrayTopic("goal").Subscribe(defaultGoal, {.periodic = 0.02, .sendAll = true});
    // Publish initial default goal
    goalPublisher.Set(defaultGoal);

    m_turretObject = m_turretField.GetObject("Turret");
    frc::SmartDashboard::PutData("Turret Field", &m_turretField);


}

void Turret::SimulationPeriodic()
{
    units::radian_t epsilon = 0.001_rad;
}

void Turret::AllowShooting()
{
    allowShooting = true;
    frc::SmartDashboard::PutBoolean("/Turret/Shooter/Allow Shooting", true);
}

void Turret::PauseShooting()
{
    allowShooting = false;
    frc::SmartDashboard::PutBoolean("/Turret/Shooter/Allow Shooting", false);
}

units::degree_t Turret::GetMeasurement()
{ // original get measurement function
    if constexpr (frc::RobotBase::IsSimulation())
    {

    }
}


void Turret::SetHood(double extension)
{
    m_hood.Set(extension);
    m_hood2.Set(extension);
}

double Turret::getTOF(double distance)
{
    return m_launchCalculator.GetTOF(distance);
}

units::meter_t Turret::getDistanceFromTOF(double TOF)
{
    return m_launchCalculator.GetDistanceFromTOF(TOF);
}

void Turret::ChangeHoodAngle(units::meter_t distance)
{
    double ballLaunchAngleDegrees = m_launchCalculator.GetHoodAngleDegrees(distance.value());
    frc::SmartDashboard::PutNumber("/Turret/Hood/Launch Angle", ballLaunchAngleDegrees);

    double servoExtention = (-(2.94699 * std::pow(10, -7)) * std::pow(ballLaunchAngleDegrees, 4) +
                             (5.89093 * std::pow(10, -5)) * std::pow(ballLaunchAngleDegrees, 3) -
                             (4.41946 * std::pow(10, -3)) * std::pow(ballLaunchAngleDegrees, 2) +
                             (0.124056) * ballLaunchAngleDegrees - 0.227319);

    frc::SmartDashboard::PutNumber("/Turret/Hood/Servo Extension", servoExtention);
    if (servoExtention > 0.8)
    {
        servoExtention = 0.8;
    }
    else if (servoExtention < 0.08)
    {
        servoExtention = 0.08;
    }
    m_hood.Set(servoExtention);
    m_hood2.Set(servoExtention);
}

double Turret::GetHoodAngle()
{
    if constexpr (frc::RobotBase::IsSimulation())
    {
        return frc::SmartDashboard::GetNumber("/Turret/Hood/Launch Angle", 0.0);
    }
    return 0.0;
}

void Turret::ChangeHoodAngle(double ballLaunchAngleDegrees)
{

    double servoExtention = (-(2.94699 * std::pow(10, -7)) * std::pow(ballLaunchAngleDegrees, 4) +
                             (5.89093 * std::pow(10, -5)) * std::pow(ballLaunchAngleDegrees, 3) -
                             (4.41946 * std::pow(10, -3)) * std::pow(ballLaunchAngleDegrees, 2) +
                             (0.124056) * ballLaunchAngleDegrees - 0.227319);

    frc::SmartDashboard::PutNumber("/Turret/Hood/Servo Extension", servoExtention);
    if (servoExtention > 0.8)
    {
        servoExtention = 0.8;
    }
    else if (servoExtention < 0.05)
    {
        servoExtention = 0.05;
    }
    frc::SmartDashboard::PutNumber("/Turret/Hood/Launch Angle", ballLaunchAngleDegrees);
    m_hood.Set(servoExtention);
    m_hood2.Set(servoExtention);
}

void Turret::ChangeHoodAngle(units::angle::radian_t ballLaunchAngle, units::meter_t distance)
{
    double hoodOffset = 0; // no offset

    ballLaunchAngle -= units::degree_t{hoodOffset};

    double ballLaunchAngleDegrees = double((ballLaunchAngle * 180) / TurretConstants::kPI);

    double servoExtention = (-(2.94699 * std::pow(10, -7)) * std::pow(ballLaunchAngleDegrees, 4) +
                             (5.89093 * std::pow(10, -5)) * std::pow(ballLaunchAngleDegrees, 3) -
                             (4.41946 * std::pow(10, -3)) * std::pow(ballLaunchAngleDegrees, 2) +
                             (0.124056) * ballLaunchAngleDegrees - 0.227319);

    frc::SmartDashboard::PutNumber("/Turret/Hood/Servo Extension", servoExtention);
    if (servoExtention > 0.8)
    {
        servoExtention = 0.8;
    }
    else if (servoExtention < 0.05)
    {
        servoExtention = 0.05;
    }
    frc::SmartDashboard::PutNumber("/Turret/Hood/Launch Angle", ballLaunchAngleDegrees);
    m_hood.Set(servoExtention);
    m_hood2.Set(servoExtention);
}

void Turret::ChangeHoodMapValue(double newOffsetValue)
{
    double distVal = frc::SmartDashboard::GetNumber("/Turret/Ballistics/Target Distance", 0);



    m_launchCalculator.AdjustHoodAngleAtDistance(distVal, newOffsetValue);
}

void Turret::ChangeLaunchSpeed(units::meters_per_second_t speed, units::meter_t distance)
{
    m_turret_shooter.SetSpeed(speed, distance);
}

void Turret::Periodic()
{
    frc::SmartDashboard::PutNumber(
        "/Turret/Comp/TurretAngleAmpDeg",
        frc::SmartDashboard::GetNumber("/Turret/Comp/TurretAngleAmpDeg", 4.0));
    frc::SmartDashboard::PutNumber(
        "/Turret/Comp/ShooterSpeedAmpRPS",
        frc::SmartDashboard::GetNumber("/Turret/Comp/ShooterSpeedAmpRPS", 0.0));

    UpdateFieldVisuals();
    double fb;
    units::volt_t ff;
    units::volt_t v;
    v = std::clamp(v, -TurretConstants::kMaxVoltage, TurretConstants::kMaxVoltage);
    if constexpr (frc::RobotBase::IsSimulation())
    {
        SimulationPeriodic();
    }
    frc::SmartDashboard::PutNumber("/Turret/Aim/Voltage", double(v));

    std::string GameData;
    GameData = frc::DriverStation::GetGameSpecificMessage();
    if (GameData.length() > 0)
    {
        switch (GameData[0])
        {
        case 'B':
            // blue case code
            break;

        case 'R':
            // red case code
            break;
        default:
            // this is corrupt data
            break;
        }
    }
    else
    {
        // code for no data recieved yet
    }
}



std::map<double, double> Turret::GetCurrentMapState()
{
    return m_launchCalculator.GetHoodAngleMap();
}

std::vector<double> Turret::manualShootingPresetMid()
{
    return std::vector<double>{25.5, 58.5, 180.0};
}

std::vector<double> Turret::manualShootingPresetLeft()
{
    return std::vector<double>{40.0, 55.0, 272.0};
}

std::vector<double> Turret::manualShootingPresetRight()
{
    return std::vector<double>{40.0, 52.0, 90.0};
}

void Turret::PresetShooting(bool temp, std::string preset)
{
    if (temp)
    {
        presetShooting = true;
    }
    else
    {
        presetShooting = false;
    }
    presetType = preset; // Sets which preset we use
}

void Turret::SetCurrentMapState(std::map<double, double> inputCurrentState)
{
    m_launchCalculator.SetHoodAngleMap(inputCurrentState);
}

void Turret::UpdateFieldVisuals()
{
    m_turretField.GetObject("Goal")->SetPose(frc::Pose3d(goal.ToMatrix()).ToPose2d());
    if (m_turretObject == nullptr)
    {
        return;
    }

    auto baseLinkPose = DoubleArrayToPose2d(baseLinkSubscriber.Get({}));
    if (baseLinkPose.has_value())
    {
        m_turretField.SetRobotPose(*baseLinkPose);
        return;
    }
}
