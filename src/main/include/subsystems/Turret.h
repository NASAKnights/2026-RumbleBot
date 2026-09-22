#pragma once

#include "frc/DataLogManager.h"
#include "wpi/DataLog.h"
#include <ctre/phoenix6/Pigeon2.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/controls/PositionVoltage.hpp>
#include <ctre/phoenix6/CANcoder.hpp>
#include <frc/DutyCycleEncoder.h>
#include <frc/Encoder.h>
#include <frc/controller/ArmFeedforward.h>
#include <frc/smartdashboard/FieldObject2d.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Quaternion.h>
#include <frc/geometry/Rotation3d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Transform3d.h>
#include <frc/geometry/Twist3d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/controller/PIDController.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/SparkMax.h>
#include <units/acceleration.h>
#include <units/angular_velocity.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/time.h>
#include <frc/DigitalInput.h>

#include <optional>
#include <string_view>
#include <vector>
#include <cmath>

#include <frc/DriverStation.h>

#include "Constants.hpp"
#include <frc/DigitalInput.h>
#include <frc/RobotBase.h>
#include <frc/Servo.h>
#include <frc/Timer.h>
#include <frc/simulation/SimDeviceSim.h>
#include <frc/simulation/SingleJointedArmSim.h>
#include <frc/simulation/DIOSim.h>
#include <networktables/DoubleArrayTopic.h>
#include <networktables/NetworkTableInstance.h>
#include <frc/smartdashboard/Field2d.h>

#include "utils/BallisticsInterpolator.h"
#include "utils/LaunchCalculator.h"
#include "utils/ballistics_rv_hub.h"
#include "utils/ballistics_rv_gnd.h"

#include "subsystems/Turret_Shooter.h"

namespace TurretConstants
{
  enum BallisticSolutionType
  {
    HUB,
    GROUND
  };

  const double kAngleP = 0.3;
  const double kAngleI = 0.00;
  const double kAngleD = 0.0; // 0.0001
  const double kIZone = 0.0;
  const auto kTurretVelLimit = units::degrees_per_second_t(600.0);
  const auto kTurretAccelLimit = units::angular_acceleration::degrees_per_second_squared_t(900); // Mech limit 27 rad/s^2(1500 degree_second_squared)
  const units::degree_t kTolerancePos = 1_deg;
  const units::degrees_per_second_t kToleranceVel = 0.5_deg_per_s;
  const int kAngleMotorId = 50;

  const auto kFFks = units::volt_t(0.2);                                // Volts static (motor)
  const auto kFFkg = units::volt_t(0.0);                                 // Volts
  const auto kFFkV = units::unit_t<frc::ArmFeedforward::kv_unit>(0.5);   // volts*s/rad
  const auto kFFkA = units::unit_t<frc::ArmFeedforward::ka_unit>(0.000); // volts*s^2/rad

  const bool kTurretEnableCurrentLimit = true;
  const int kTurretContinuousCurrentLimit = 35;
  const int kTurretPeakCurrentLimit = 60;
  const double kTurretPeakCurrentDuration = 0.1;

  const std::array<double, 2> kSimNoise = {0.0};
  const frc::DCMotor kSimMotor = frc::DCMotor::KrakenX60(1);
  const double kGearRatio = 86.61; // gear ratio for motor to arm
  const units::moment_of_inertia::kilogram_square_meter_t kmoi =
      units::moment_of_inertia::kilogram_square_meter_t(0.06742); // I = MR^2
  const units::length::meter_t kTurretRadius = units::length::meter_t(0.3048);
  const units::mass::kilogram_t kTurretMass = units::mass::kilogram_t(2.26796);
  
  const bool kGravity = false;
  const units::angle::radian_t kTurretStartAngle = units::angle::radian_t(0.0);


  const units::radian_t kLeadAngleClamp = 0.698_rad;

  //HOOD VALUES
  const frc::DCMotor kHoodSimMotor = frc::DCMotor::KrakenX60(1);
  const double kHoodGearRatio = 1; // gear ratio for motor to arm
  const units::moment_of_inertia::kilogram_square_meter_t kHoodmoi =
      units::moment_of_inertia::kilogram_square_meter_t(0.08428); // I = MR^2
  const units::length::meter_t kHoodRadius = units::length::meter_t(0.3048);
  const units::mass::kilogram_t kHoodMass = units::mass::kilogram_t(0.907185);
  const units::angle::radian_t kHoodStartAngle = units::angle::radian_t(0.0);
  const units::angle::radian_t kHoodMinAngle = 35_deg; //Needs to increase to fix skew to the right
  const units::angle::radian_t kHoodMaxAngle = 70_deg;
  const double kHoodXOffset = 0.1;
  const double kHoodYOffset = 0.0;


  const double turretPositionConversionFactor  = 360.0 / TurretConstants::kGearRatio;
  const double turretVelocityConversionFactor = 360.0 / TurretConstants::kGearRatio / 60.0;

  const double kXOffset =  -0.181;  // (m) 7.3 inches from center of robot (opposite intake)
  const double kYOffset = 0.18;  // (m) 7.3 inches from center of robot
  const double kZOffset = 0.0;
  const units::degree_t kAngleOffset(0.0);
  const units::volt_t kMaxVoltage = 6.0_V; 
  
  // HOMING will find the kminAngle limit switch and set the encoder position to this
  // value.  kminAngle should therefore be set based on the physical location of the
  // limit switch, such that 0 deg will point the turret directly away from the intake, 
  // orthogonal to the robot. 
 
  const std::vector<double> BlueHubCoords = {4.625594, 4.034536, 1.829}; //in meters
  const std::vector<double> TopBlueCoords = {1.11252, 6.930136, 0.0};
  const std::vector<double> BottomBlueCoords = {1.11252, 2.010664, 0.0};

  const std::vector<double> RedHubCoords = {11.915394, 4.034536, 1.829};
  const std::vector<double> TopRedCoords = {15.428468, 6.930136, 0.0};
  const std::vector<double> BottomRedCoords = {15.428468, 2.010664, 0.0};

  const units::length::meter_t BlueAllianceZoneX = 4.625594_m;
  const units::length::meter_t MidFieldLine = 4.034536_m;
  const units::length::meter_t RedAllianceZoneX = 11.915394_m;

  const double kPI = 3.14159265358979323846;

  const units::angle::degree_t kHoodFlattenAngle = 90_deg;
} // namespace TurretConstants

/**
 * A robot m_arm subsystem that moves with a motion profile.
 */
class Turret : public frc2::SubsystemBase
{

public:
  Turret();
  void Periodic();
  void Emergency_Stop();
  void UseOutput();
  void SimulationPeriodic();
  void AllowShooting();
  void PresetShooting(bool temp, std::string preset);
  void PauseShooting();
  void Enable();
  void Disable();
  void UpdateTurretGoal(const frc::Pose2d &robotPose);
  void Reset()
  {
    m_controller.Reset();
  }

  void ChangeHoodAngle(units::angle::radian_t launchAngle, units::meter_t distance);
  void ChangeHoodAngle(units::meter_t distance);
  void ChangeHoodAngle(double ballLaunchAngleDegrees);

  void SetHood(double extension);
  double getTOF(double distance);
  units::meter_t getDistanceFromTOF(double TOF);
  double GetRobotVelocityShooterSpeedCorrection(double tn);
  double GetHoodAngle();
  void ChangeLaunchSpeed(units::meters_per_second_t speed, units::meter_t distance);

  std::map<double, double> GetCurrentMapState();
  void ChangeHoodMapValue(double newValue);
  void SaveLaunchMapToFile() const
  {
    m_launchCalculator.SaveToFile();
  }
  void PublishLaunchMap() const
  {
    m_launchCalculator.PublishCurrentTable();
  }

  void SetCurrentMapState(std::map<double, double> inputCurrentState);
  std::vector<double> manualShootingPresetMid();
  std::vector<double> manualShootingPresetLeft();
  std::vector<double> manualShootingPresetRight();

  // void get_pigeon();
  units::degree_t GetMeasurement();
  units::degrees_per_second_t GetVelocity();
  bool Flatten = false;
  Turret_Shooter m_turret_shooter; //made public need to access in robot.cpp
  
  // units::time::second_t time_brake_released;

private:
  frc::Transform3d goal = frc::Transform3d(2_m, 2_m, 0_m, frc::Rotation3d());
  nt::DoubleArrayPublisher goalPublisher;
  nt::DoubleArraySubscriber goalSubscriber;
  static std::optional<frc::Pose2d> DoubleArrayToPose2d(const std::vector<double> &arr)
  {
    if (arr.size() < 7)
    {
      return std::nullopt;
    }

    auto x = units::length::meter_t(arr.at(0));
    auto y = units::length::meter_t(arr.at(1));

    auto o = units::angle::radian_t(
        frc::Rotation3d(frc::Quaternion(arr.at(6),
                                        arr.at(3),
                                        arr.at(4),
                                        arr.at(5)))
            .ToRotation2d()
            .Radians()
            .value());

    return frc::Pose2d(x, y, o);
  }
  void UpdateFieldVisuals();

  frc::Servo m_hood{7};
  frc::Servo m_hood2{8};
  
  frc::ArmFeedforward m_feedforward;
    wpi::log::DoubleLogEntry m_AngleLog;
    wpi::log::DoubleLogEntry m_SetPointLog;
    wpi::log::IntegerLogEntry m_StateLog;
    wpi::log::DoubleLogEntry m_MotorCurrentLog;
    wpi::log::DoubleLogEntry m_MotorVoltageLog;
  frc::Timer *m_timer;
  float Turret_Angle;

  units::degree_t m_goal;
  frc::Timer m_simTimer;

  // frc::sim::SingleJointedArmSim m_HoodSim;

  frc::PIDController m_controller;
  units::degrees_per_second_t m_velocityGoal{0.0};

  nt::DoubleArraySubscriber baseLinkSubscriber;
  std::string_view robotPoseLink = "base_link";
  std::string_view goalPoseLink = "goal";
  std::vector<frc::Pose2d> poses{};
  nt::NetworkTableInstance networkTableInst;
  frc::Field2d m_turretField;
  frc::FieldObject2d *m_turretObject = nullptr;
  
  // Pose staleness detection
  int64_t m_lastPoseUpdateTime = 0;
  std::optional<frc::Pose2d> m_lastValidPose;
  wpi::log::BooleanLogEntry m_PoseStaleLog;


  bool allowShooting = false;
  bool presetShooting = false;

  std::string presetType = "middle";
  LaunchCalculator m_launchCalculator;

  std::map<double, double> kHoodOffsetMap = {
      {1.0, 0.},
      {2.0, 4.},
      {2.5, 7.},
      {3.0, 10.}, //8 deg extra
      {4.0, 12.}, //10 deg extra
      {5.0, 13.},
      {6.0, 16.},
      {7.0, 19.}
  };

  std::map<double, double> kShotTOFMap = {
      {1.0, 1.2574},
      {1.5, 1.3077},
      {2.0, 1.3580},
      {2.5, 1.4083},
      {3.0, 1.4586},
      {3.5, 1.5089},
      {4.0, 1.5592},
      {4.5, 1.6095},
      {5.0, 1.6598},
      {5.5, 1.8},
      {6.0, 1.85},
      {6.5, 1.9},
      {7.0, 2.0}
  };

  std::map<double, double> kDistanceFromTOFMap = {
      {1.2574, 1.0},
      {1.3077, 1.5},
      {1.3580, 2.0},
      {1.4083, 2.5},
      {1.4586, 3.0},
      {1.5089, 3.5},
      {1.5592, 4.0},
      {1.6095, 4.5},
      {1.6598, 5.0},
      {1.8, 5.5},
      {1.85, 6.0},
      {1.9, 6.5},
      {2.0, 7.0}
  };

  std::vector<std::pair<double, double>> kHoodAngleVector;

};
