# Drivebase-Only Trim Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Reduce the robot program to the drivebase and its supporting infrastructure, and consolidate 2026 game/field information into a new `FieldData` class.

**Architecture:** Three staged commits, top-down. Gut `Robot` of scoring-mechanism references first (definitions still exist, so the compiler names dangling references precisely), then extract `FieldData`, then delete the now-unreferenced files. Each stage ends at a tree that compiles and simulates cleanly.

**Tech Stack:** C++20, WPILib 2026 / GradleRIO 2026.2.1, CTRE Phoenix 6, PhotonVision, PathPlannerLib, GoogleTest.

**Spec:** `docs/superpowers/specs/2026-09-16-drivebase-only-trim-design.md`

## Global Constraints

- `JAVA_HOME` must be exported before any Gradle command: `export JAVA_HOME="/c/Users/Public/wpilib/2026/jdk"`
- Never use `-x test`. C++ has no `test` task; it fails the build outright.
- Never pipe Gradle into `tail`/`head` — the pager replaces Gradle's exit code.
- A build is only valid evidence if it reports `N actionable tasks: N executed`. A cached `UP-TO-DATE` proves nothing.
- No behavior change to the drivebase. No PID/feedforward re-tuning.
- Swerve module offsets in `Constants.hpp` stay zero. Calibration is hardware bring-up, out of scope.
- **No new unit tests.** The spec explicitly chose "pure relocation + bug fix" over the tests variant. Verification is Gate A + Gate B per task. `NetworkTableMapTest` must keep passing as a regression check.

### Verification gates (run at the end of every task)

**Gate A — Build**

```bash
export JAVA_HOME="/c/Users/Public/wpilib/2026/jdk"
./gradlew clean build --console=plain > /tmp/gateA.log 2>&1; echo "EXIT=$?"
grep -E "^BUILD|actionable" /tmp/gateA.log
grep -cE " error: " /tmp/gateA.log
```

Pass: `EXIT=0`, `BUILD SUCCESSFUL`, `N actionable tasks: N executed`, error count `0`.

**Gate B — Simulation smoke test**

```bash
export JAVA_HOME="/c/Users/Public/wpilib/2026/jdk"
timeout 60 ./gradlew simulateNativeRelease --console=plain > /tmp/gateB.log 2>&1; echo "EXIT=$?"
grep -c "Robot program startup complete" /tmp/gateB.log
grep -icE "unhandled|terminate called|segmentation|Aborted|what\(\):" /tmp/gateB.log
grep -oE "Error at [A-Za-z:_]+" /tmp/gateB.log | sort -u
git status --short
```

Pass: `EXIT=124` (stayed alive), startup marker count `1`, crash count `0`, the only `Error at` category is `PrintLoopOverrunMessage`, **and `git status` is clean**.

> **Data-loss warning for Tasks 1 and 2.** On pre-Task-3 code, `DisabledInit()` calls `m_turret.SaveLaunchMapToFile()`, which writes an empty table over `src/main/deploy/LaunchCalculator_Points.csv` in simulation, destroying all 15 rows of shooter tuning. If `git status` shows that file modified after Gate B, run `git checkout -- src/main/deploy/LaunchCalculator_Points.csv` before committing. This stops being possible after Task 3.

### Pre-existing dead code — knowingly left alone

Do **not** remove these. They are dead but unrelated to scoring mechanisms, and removing them inflates the diff beyond the approved scope: `autoMap`, `baseLink`, `targetKey`, `prevAuto`, `m_pathfind`, `scoreClosest`, `m_DebugController`, and the `<photon/PhotonUtils.h>` include in `Robot.hpp`.

---

## File Structure

| File | Action | Responsibility after this plan |
| --- | --- | --- |
| `src/main/include/Robot.hpp` | Modify | Declares the drivebase-only robot: SwerveDrive, driver joystick, PDH, POI, auto chooser |
| `src/main/cpp/Robot.cpp` | Modify | Robot lifecycle, drivebase default command, 4 driver bindings |
| `src/main/include/subsystems/FieldData.h` | Create | 2026 field geometry and game-state logic |
| `src/main/cpp/subsystems/FieldData.cpp` | Create | `CheckActiveHub` implementation |
| `src/main/include/Constants.hpp` | Modify | Electrical/Drive/Module/MathUtilNK only |
| 22 subsystem/command/ballistics files | Delete | — |
| `src/main/deploy/LaunchCalculator_Points.csv` | Delete | — |

---

## Task 1: Gut `Robot` of scoring-mechanism references

**Files:**
- Modify: `src/main/include/Robot.hpp` (154 lines → ~95)
- Modify: `src/main/cpp/Robot.cpp` (518 lines → ~200)

**Interfaces:**
- Consumes: nothing from earlier tasks.
- Produces: a `Robot` class with no reference to `Turret`, `Turret_Shooter`, `TurretIntake`, `Wrist`, `LEDController`, or the five removed commands. `Robot::CheckActiveHub()` still exists and still returns `std::string` — Task 2 moves it.

- [ ] **Step 1: Replace `src/main/include/Robot.hpp` entirely**

```cpp
// Copyright (c) FRC Team 122. All Rights Reserved.

#pragma once

#include <optional>

#include "frc/DataLogManager.h"
#include "wpi/DataLog.h"
#include <frc/Joystick.h>
#include <frc/PowerDistribution.h>
#include <frc/TimedRobot.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/AnalogInput.h>
#include "utils/POIGenerator.h"
#include <photon/PhotonUtils.h>
#include <memory>

#include <ctre/phoenix6/CANBus.hpp>

#include <frc2/command/CommandPtr.h>
#include <frc2/command/CommandScheduler.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/button/Trigger.h>

#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>

#include <units/angular_velocity.h>
#include <units/velocity.h>

#include "subsystems/SwerveDrive.hpp"

#include "commands/AutoWheelOffsets.h"

#include <cmath>

class Robot : public frc::TimedRobot
{
public:
    Robot();

    //
    // Robot Schedule methods
    //
    void RobotInit() override;
    void RobotPeriodic() override;
    void DisabledInit() override;
    void DisabledPeriodic() override;
    void AutonomousInit() override;
    void AutonomousPeriodic() override;
    void AutonomousExit() override;
    void TeleopInit() override;
    void TeleopPeriodic() override;
    void TeleopExit() override;
    void TestPeriodic() override;
    void SimulationInit() override;
    void SimulationPeriodic() override;

    std::string CheckActiveHub();

private:
    // Have it empty by default so that if testing teleop it
    // doesn't have undefined behavior and potentially crash.
    std::optional<frc2::CommandPtr> m_autonomousCommand;

    std::map<int, std::pair<pathplanner::PathPlannerAuto, frc::Pose2d>> autoMap;

    // Subsystems

    frc::SendableChooser<std::string> m_chooser;
    frc::AnalogInput batteryShunt{0};

    ctre::phoenix6::CANBus NKCANBus{"NKCANivore"};
    SwerveDrive m_swerveDrive{NKCANBus};

    std::string_view baseLink = "base_link";

    std::string targetKey = "POI/Calibration POIs";
    std::string prevAuto = "";

    frc::PowerDistribution m_pdh =
        frc::PowerDistribution{1, frc::PowerDistribution::ModuleType::kRev};

    // PS4 controllers
    frc::Joystick m_driverController{DriveConstants::kDriverPort};
    frc::Joystick m_DebugController{DriveConstants::kDebugControllerPort};

    // Power Distribution
    wpi::log::DoubleLogEntry m_VoltageLog;
    wpi::log::DoubleLogEntry m_CurrentLog;
    wpi::log::DoubleLogEntry m_PowerLog;
    wpi::log::DoubleLogEntry m_EnergyLog;
    wpi::log::DoubleLogEntry m_TemperatureLog;
    wpi::log::DoubleLogEntry m_BatteryLog;
    frc::SendableChooser<frc2::Command *> autoChooser;
    POIGenerator m_poiGenerator;
    frc2::CommandPtr m_pathfind = frc2::InstantCommand().ToPtr();
    frc2::CommandPtr scoreClosest = frc2::InstantCommand().ToPtr();

    frc2::CommandPtr addPOICommand = frc2::CommandPtr(frc2::InstantCommand([this]
                                                                           { return m_poiGenerator.MakePOI(); }))
                                         .IgnoringDisable(true);

    frc2::CommandPtr removePOICommand = frc2::CommandPtr(frc2::InstantCommand([this]
                                                                              { return m_poiGenerator.RemovePOI(); }))
                                            .IgnoringDisable(true);

    frc2::CommandPtr autoWheelOffsetsCommand = AutoWheelOffsets(&m_swerveDrive).ToPtr().IgnoringDisable(true);

    // Robot Container methods
    void CreateRobot();
    void BindCommands();

    frc::Pose2d autoStartPose;

    frc2::CommandPtr GetAutonomousCommand();
    void SetAutonomousCommand(std::string a);
    void UpdateDashboard();
    frc::EventLoop m_POVloop{};
};
```

Changes made: dropped includes for `Wrist.h`, `Turret.h`, `TurretIntake.h`, `LEDController.h`, `NamedCommands.h`, `POVButton.h`, and the five command headers; dropped members `m_wrist`, `m_turret`, `m_intake`, `m_led`, `m_operatorController`, `testingRotation`, the six `nt::` publishers, and `networkTableInst`.

`networkTableInst` goes because its only two uses were creating the `SmartDashboard` table for `modelPosePublisher`. It is orphaned by a spec-mandated removal, so it goes with it.

- [ ] **Step 2: Replace `src/main/cpp/Robot.cpp` entirely**

```cpp
// Copyright (c) FRC Team 122. All Rights Reserved.

#include "Robot.hpp"

#include <exception>

#include <frc/Errors.h>

Robot::Robot()
{
    this->CreateRobot();
}

// This function is called during startup
void Robot::RobotInit()
{
    frc::DataLogManager::Start();
    wpi::log::DataLog &log = frc::DataLogManager::GetLog();

    m_VoltageLog = wpi::log::DoubleLogEntry(log, "/PDP/Voltage");
    m_CurrentLog = wpi::log::DoubleLogEntry(log, "/PDP/Current");
    m_PowerLog = wpi::log::DoubleLogEntry(log, "/PDP/Power");
    m_EnergyLog = wpi::log::DoubleLogEntry(log, "/PDP/Energy");
    m_TemperatureLog = wpi::log::DoubleLogEntry(log, "/PDP/Temperature");
    m_BatteryLog = wpi::log::DoubleLogEntry(log, "Robot/Battery");

    frc::SmartDashboard::PutString("POIName", "");
    frc::SmartDashboard::PutData("AddPOI", addPOICommand.get());
    frc::SmartDashboard::PutData("RemovePOI", removePOICommand.get());
    frc::SmartDashboard::PutData("Set", autoWheelOffsetsCommand.get());

    try
    {
        autoChooser = pathplanner::AutoBuilder::buildAutoChooser();
    }
    catch (const std::exception& e)
    {
        FRC_ReportWarning("Failed to load PathPlanner autos: {}", e.what());
    }
    catch (...)
    {
        FRC_ReportWarning("Failed to load PathPlanner autos: unknown error");
    }

    frc::SmartDashboard::PutData("Auto Chooser", &autoChooser);
}

// This function is called every 20 ms
void Robot::RobotPeriodic()
{
    frc2::CommandScheduler::GetInstance().Run();
    this->UpdateDashboard();
    m_POVloop.Poll();

    // The switchable channel powered the turret shooter, which no longer
    // exists. Keep it off.
    m_pdh.SetSwitchableChannel(false);

    m_VoltageLog.Append(m_pdh.GetVoltage());
    m_CurrentLog.Append(m_pdh.GetTotalCurrent());
    m_PowerLog.Append(m_pdh.GetTotalPower());
    m_EnergyLog.Append(m_pdh.GetTotalEnergy());
    m_TemperatureLog.Append(m_pdh.GetTemperature());
    m_BatteryLog.Append(batteryShunt.GetVoltage());
}

// This function is called once each time the robot enters Disabled mode.
void Robot::DisabledInit()
{
    if constexpr (frc::RobotBase::IsSimulation())
    {
        m_swerveDrive.ResetPose(frc::Pose2d());
        m_swerveDrive.ResetDriveEncoders();
    }
}

void Robot::SetAutonomousCommand(std::string a)
{

}

void Robot::AutonomousInit()
{
    auto m_autonomousCommand = autoChooser.GetSelected();
    m_swerveDrive.ResetPose(autoStartPose);

    if (m_autonomousCommand)
    {
        m_autonomousCommand->Schedule();
    }
}

void Robot::AutonomousPeriodic() {}

void Robot::AutonomousExit()
{
    m_swerveDrive.InvertHeading();
}

void Robot::TeleopInit()
{
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand)
    {
        m_autonomousCommand->Cancel();
    }
    m_swerveDrive.TurnVisionOn(); // Turn Vision back on for Teleop
}

void Robot::TeleopPeriodic()
{
}

void Robot::TeleopExit()
{
}

/**
 * This function is called periodically during test mode.
 */
void Robot::TestPeriodic() {}

/**
 * This function is called once when the robot is first started up.
 */
void Robot::SimulationInit() {}

/**
 * This function is called periodically whilst in simulation.
 */
void Robot::SimulationPeriodic() {}

/**
 * Initializes the robot subsystems and binds commands
 */
void Robot::CreateRobot()
{
    m_swerveDrive.SetDefaultCommand(frc2::RunCommand(
        [this]
        {
            auto controllerIn = m_driverController.GetRawButton(4);

            auto leftXAxis = MathUtilNK::calculateAxis(m_driverController.GetRawAxis(1),
                                                       DriveConstants::kDefaultAxisDeadband);
            auto leftYAxis = MathUtilNK::calculateAxis(m_driverController.GetRawAxis(0),
                                                       DriveConstants::kDefaultAxisDeadband);
            auto rightXAxis = MathUtilNK::calculateAxis(m_driverController.GetRawAxis(4),
                                                        DriveConstants::kDefaultAxisDeadband);

            if (controllerIn)
                // Robot-Oriented Drive
                m_swerveDrive.Drive(frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                    -leftXAxis * 1.0_mps,
                    -leftYAxis * 1.0_mps,
                    -rightXAxis * 2.0_rad_per_s, m_swerveDrive.GetHeading()));
            else
            {
                m_swerveDrive.Drive(frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                    -leftXAxis * DriveConstants::kMaxTranslationalVelocity,
                    -leftYAxis * DriveConstants::kMaxTranslationalVelocity,
                    -rightXAxis * DriveConstants::kMaxRotationalVelocity, m_swerveDrive.GetHeading()));
            }
        },
        {&m_swerveDrive}));

    // Configure the button bindings
    BindCommands();
    m_swerveDrive.ResetHeading();
}

/**
 * Binds commands to Joystick buttons
 */
void Robot::BindCommands()
{

    // --------------DRIVER BUTTONS----------------------------------
    frc2::JoystickButton(&m_driverController, 3)
        .OnTrue(frc2::CommandPtr(
            frc2::InstantCommand([this]
                                 { return m_swerveDrive.ResetHeading(); })));

    frc::BooleanEvent downPOVDriverBE = frc::BooleanEvent(
        &m_POVloop,
        [&controller = m_driverController]{
            return (controller.GetPOV()>=135) && (controller.GetPOV()<=225);
        }
    ).Debounce(0.2_s);

    frc2::Trigger POVDownTrigBR = downPOVDriverBE.CastTo<frc2::Trigger>();
    POVDownTrigBR.WhileTrue(
                    frc2::CommandPtr(frc2::RunCommand([this] {
                        m_swerveDrive.MakeX(true);
                    })))
                .OnFalse(
                    frc2::CommandPtr(frc2::InstantCommand([this] {
                        m_swerveDrive.MakeX(false);
                    })));
}

void Robot::DisabledPeriodic()
{
    std::string poiName = std::string("POI/") + frc::SmartDashboard::GetString("POIName", "");
    frc::SmartDashboard::PutBoolean("IsPersist", frc::SmartDashboard::IsPersistent(poiName));
}

void Robot::UpdateDashboard()
{
    frc::SmartDashboard::PutNumber("Robot/Battery Amps", batteryShunt.GetVoltage());
    frc::SmartDashboard::PutNumber("Robot/PDH Total Current", m_pdh.GetTotalCurrent());
}

std::string Robot::CheckActiveHub()
{
    std::string GameData;
    std::string AutoWinner;
    GameData = frc::DriverStation::GetGameSpecificMessage();
    if(GameData.length() > 0)
    {
        switch (GameData[0])
        {
            case 'B' :
                AutoWinner = "Blue";
                break;
            case 'R' :
                AutoWinner = "Red";
                break;
            default :
                AutoWinner = "None";
                break;
        }
    } else {
        //code for no data recieved yet
        AutoWinner = "None";
    }

    units::time::second_t matchtimer = frc::DriverStation::GetMatchTime();
    units::time::second_t startOfMatch = units::time::second_t{160};
    units::time::second_t endOfAuto = units::time::second_t{140};
    units::time::second_t endOfTransition = units::time::second_t{130};
    units::time::second_t endOfPeriod1 = units::time::second_t{105};
    units::time::second_t endOfPeriod2 = units::time::second_t{80};
    units::time::second_t endOfPeriod3 = units::time::second_t{55};
    units::time::second_t endOfPeriod4 = units::time::second_t{30};
    units::time::second_t endOfEndgame = units::time::second_t{0};
    frc::DriverStation::Alliance AllianceColor = frc::DriverStation::GetAlliance().value();

    //actual one:
    if ((matchtimer < endOfAuto && matchtimer >= endOfTransition) || (matchtimer < endOfPeriod4 && matchtimer >= endOfEndgame))
    {
        if (AllianceColor == frc::DriverStation::Alliance::kBlue)
        {
            return "BlueActive";
        }
        else
        {
            return "RedActive";
        }

    }
    else if(GameData == "R" || GameData == "B")
    {
        if ((matchtimer < endOfTransition && matchtimer >= endOfPeriod1) || (matchtimer < endOfPeriod2 && matchtimer >= endOfPeriod3))
        {
            if (AutoWinner == "Blue")
            {
                return "RedActive";
            }
            else
            {
                return "BlueActive";
            }

        }
        else if ((matchtimer < endOfPeriod1 && matchtimer >= endOfPeriod2) || (matchtimer < endOfPeriod3 && matchtimer >= endOfPeriod4))
        {
            if (AutoWinner == "Blue")
            {
                return "BlueActive";
            }
            else
            {
                return "RedActive";
            }

        }
    }
    else
    {
        if (AllianceColor == frc::DriverStation::Alliance::kBlue)
        {
            return "BlueActive";
        }
        else
        {
            return "RedActive";
        }

    }
}


#ifndef RUNNING_FRC_TESTS
int main()
{
    return frc::StartRobot<Robot>();
}
#endif
```

`CheckActiveHub` is copied verbatim here, still carrying its missing-return defect. Task 2 moves it and fixes it. Keeping it unchanged in this task keeps each commit to one concern.

- [ ] **Step 3: Run Gate A**

Run the Gate A block from Global Constraints.
Expected: `EXIT=0`, `BUILD SUCCESSFUL`, `19 actionable tasks: 19 executed`, error count `0`.

Expect one **new** warning, which is correct and expected: `CheckActiveHub` now produces `control reaches end of non-void function` where it previously did too. Confirm the count of `-Wreturn-type` warnings is unchanged at 2 (`GetHeading` and `CheckActiveHub`):

```bash
grep -c "Wreturn-type" /tmp/gateA.log
```

- [ ] **Step 4: Run Gate B**

Run the Gate B block from Global Constraints.
Expected: `EXIT=124`, startup marker `1`, crash count `0`, only `PrintLoopOverrunMessage`.

The baseline `SparkBase::SetInverted` warning should now be **absent** — REVLib devices lived only in `Turret`, `Turret_Shooter`, and `Wrist`, which `Robot` no longer constructs. Confirm:

```bash
grep -c "SparkBase::SetInverted" /tmp/gateB.log
```
Expected: `0`.

If `git status` shows `LaunchCalculator_Points.csv` modified, revert it:

```bash
git checkout -- src/main/deploy/LaunchCalculator_Points.csv
```

- [ ] **Step 5: Commit**

```bash
git add src/main/include/Robot.hpp src/main/cpp/Robot.cpp
git commit -m "Remove scoring-mechanism references from Robot

Drop the Turret, Wrist, TurretIntake and LEDController members, their
includes, the six PathPlanner NamedCommands registrations, the wrist and
turret AddPeriodic callbacks, and all operator-controller bindings.

Remove the 3D model pose publishing block and the five Pose3d publishers
that were declared but never used. networkTableInst goes with it; its only
use was building the SmartDashboard table for the model poses.

Pin the PDH switchable channel off, since the turret shooting flag that
drove it no longer has a writer.

The subsystem and command files still exist and still compile; they are
deleted in a later commit."
```

---

## Task 2: Extract `FieldData`

**Files:**
- Create: `src/main/include/subsystems/FieldData.h`
- Create: `src/main/cpp/subsystems/FieldData.cpp`
- Modify: `src/main/include/Constants.hpp` (remove the `FieldConstants` namespace, lines 229-253)
- Modify: `src/main/include/Robot.hpp` (remove the `CheckActiveHub` declaration)
- Modify: `src/main/cpp/Robot.cpp` (remove the `CheckActiveHub` definition)

**Interfaces:**
- Consumes: the `Robot` produced by Task 1.
- Produces: `class FieldData` with `std::string CheckActiveHub()`, the nested `struct FieldZone` with `bool IsInside(frc::Translation2d) const`, and static members `kBlueHub`, `kRedHub`, `kBlueAllianceZone`, `kRedAllianceZone`, `kNeutralZone`, `kBlueNeutralZone`, `kRedNeutralZone`. **Nothing calls it** — see the note below.

> `FieldData` is intentionally unreferenced after this task, exactly like `Sharp_IRDistanceSensor`. It preserves live 2026 game logic in the shape the xUML model calls for, ready for `Robot State` and `Autonomous` to consume during the rebuild. It compiles as part of the build so it cannot silently rot. Do not delete it as dead code.

- [ ] **Step 1: Create `src/main/include/subsystems/FieldData.h`**

```cpp
// Copyright (c) FRC Team 122. All Rights Reserved.

#pragma once

#include <string>

#include <frc/DriverStation.h>
#include <frc/geometry/Translation2d.h>
#include <units/length.h>
#include <units/time.h>

/**
 * 2026 field geometry and game-state logic.
 *
 * Holds no hardware and is deliberately not a SubsystemBase: it needs no
 * scheduler slot. Relocated from Robot and the FieldConstants namespace so
 * that field and game data has a single home, per the project model.
 */
class FieldData
{
public:
    /** An axis-aligned rectangular region of the field. */
    struct FieldZone
    {
        frc::Translation2d min;
        frc::Translation2d max;

        // Helper function to check if a robot is inside this zone
        bool IsInside(frc::Translation2d point) const
        {
            return point.X() >= min.X() && point.X() <= max.X() &&
                   point.Y() >= min.Y() && point.Y() <= max.Y();
        }
    };

    /**
     * Returns which alliance's hub is currently active, as "BlueActive" or
     * "RedActive".
     *
     * Returns "None" when the match state does not determine an active hub.
     */
    std::string CheckActiveHub();

    static const frc::Translation2d kBlueHub;
    static const frc::Translation2d kRedHub;

    static const FieldZone kBlueAllianceZone;
    static const FieldZone kRedAllianceZone;
    static const FieldZone kNeutralZone;

    static const FieldZone kBlueNeutralZone;
    static const FieldZone kRedNeutralZone;
};
```

- [ ] **Step 2: Create `src/main/cpp/subsystems/FieldData.cpp`**

The `CheckActiveHub` body is identical to the original except for the final
`return "None";`, which replaces the path that previously fell off the end of
the function.

```cpp
// Copyright (c) FRC Team 122. All Rights Reserved.

#include "subsystems/FieldData.h"

// Explicit units::meter_t rather than the _m literal suffix: the literal lives
// in the units::literals namespace, which Constants.hpp happened to have in
// scope transitively. Spelling the type out removes that dependency.
const frc::Translation2d FieldData::kBlueHub{units::meter_t{4.625594}, units::meter_t{4.034536}};
const frc::Translation2d FieldData::kRedHub{units::meter_t{11.915394}, units::meter_t{4.034536}};

const FieldData::FieldZone FieldData::kBlueAllianceZone{
    {units::meter_t{0.0}, units::meter_t{0.0}},
    {units::meter_t{4.625594}, units::meter_t{8.069326}}};
const FieldData::FieldZone FieldData::kRedAllianceZone{
    {units::meter_t{11.915394}, units::meter_t{0.0}},
    {units::meter_t{16.540988}, units::meter_t{8.069326}}};
const FieldData::FieldZone FieldData::kNeutralZone{
    {units::meter_t{4.625594}, units::meter_t{0.0}},
    {units::meter_t{11.915394}, units::meter_t{8.069326}}};

const FieldData::FieldZone FieldData::kBlueNeutralZone{
    {units::meter_t{4.625594}, units::meter_t{0.0}},
    {units::meter_t{16.540988}, units::meter_t{8.069326}}};
const FieldData::FieldZone FieldData::kRedNeutralZone{
    {units::meter_t{0.0}, units::meter_t{0.0}},
    {units::meter_t{11.915394}, units::meter_t{8.069326}}};

std::string FieldData::CheckActiveHub()
{
    std::string GameData;
    std::string AutoWinner;
    GameData = frc::DriverStation::GetGameSpecificMessage();
    if (GameData.length() > 0)
    {
        switch (GameData[0])
        {
            case 'B':
                AutoWinner = "Blue";
                break;
            case 'R':
                AutoWinner = "Red";
                break;
            default:
                AutoWinner = "None";
                break;
        }
    }
    else
    {
        // code for no data recieved yet
        AutoWinner = "None";
    }

    units::time::second_t matchtimer = frc::DriverStation::GetMatchTime();
    units::time::second_t endOfAuto = units::time::second_t{140};
    units::time::second_t endOfTransition = units::time::second_t{130};
    units::time::second_t endOfPeriod1 = units::time::second_t{105};
    units::time::second_t endOfPeriod2 = units::time::second_t{80};
    units::time::second_t endOfPeriod3 = units::time::second_t{55};
    units::time::second_t endOfPeriod4 = units::time::second_t{30};
    units::time::second_t endOfEndgame = units::time::second_t{0};
    frc::DriverStation::Alliance AllianceColor = frc::DriverStation::GetAlliance().value();

    if ((matchtimer < endOfAuto && matchtimer >= endOfTransition) ||
        (matchtimer < endOfPeriod4 && matchtimer >= endOfEndgame))
    {
        if (AllianceColor == frc::DriverStation::Alliance::kBlue)
        {
            return "BlueActive";
        }
        else
        {
            return "RedActive";
        }
    }
    else if (GameData == "R" || GameData == "B")
    {
        if ((matchtimer < endOfTransition && matchtimer >= endOfPeriod1) ||
            (matchtimer < endOfPeriod2 && matchtimer >= endOfPeriod3))
        {
            if (AutoWinner == "Blue")
            {
                return "RedActive";
            }
            else
            {
                return "BlueActive";
            }
        }
        else if ((matchtimer < endOfPeriod1 && matchtimer >= endOfPeriod2) ||
                 (matchtimer < endOfPeriod3 && matchtimer >= endOfPeriod4))
        {
            if (AutoWinner == "Blue")
            {
                return "BlueActive";
            }
            else
            {
                return "RedActive";
            }
        }
    }
    else
    {
        if (AllianceColor == frc::DriverStation::Alliance::kBlue)
        {
            return "BlueActive";
        }
        else
        {
            return "RedActive";
        }
    }

    // Previously fell off the end of a non-void function, returning an
    // indeterminate value, when the game message was "R" or "B" but the match
    // timer sat outside every window above.
    return "None";
}
```

The unused `startOfMatch` local from the original is dropped; it was assigned and never read.

- [ ] **Step 3: Remove the `FieldConstants` namespace from `Constants.hpp`**

Delete lines 229-253 — the entire `namespace FieldConstants { ... }` block, from `namespace FieldConstants {` through its closing `}`. Leave everything above it intact. The file should now end after the `MathUtilNK` namespace's closing brace.

- [ ] **Step 4: Remove `CheckActiveHub` from `Robot`**

In `src/main/include/Robot.hpp`, delete this line and the blank line following it:

```cpp
    std::string CheckActiveHub();
```

In `src/main/cpp/Robot.cpp`, delete the entire `std::string Robot::CheckActiveHub() { ... }` definition, from its opening line through its closing brace, leaving `UpdateDashboard` and the `#ifndef RUNNING_FRC_TESTS` block intact.

- [ ] **Step 5: Run Gate A**

Run the Gate A block from Global Constraints.
Expected: `EXIT=0`, `BUILD SUCCESSFUL`, `N actionable tasks: N executed`, error count `0`.

Confirm the `-Wreturn-type` warning count dropped from 2 to 1 — `CheckActiveHub` is fixed, `GetHeading` remains (out of scope, spec §11):

```bash
grep -c "Wreturn-type" /tmp/gateA.log
```
Expected: `1`.

- [ ] **Step 6: Run Gate B**

Run the Gate B block from Global Constraints.
Expected: `EXIT=124`, startup marker `1`, crash count `0`, only `PrintLoopOverrunMessage`, and revert `LaunchCalculator_Points.csv` if it shows modified.

- [ ] **Step 7: Commit**

```bash
git add src/main/include/subsystems/FieldData.h src/main/cpp/subsystems/FieldData.cpp src/main/include/Constants.hpp src/main/include/Robot.hpp src/main/cpp/Robot.cpp
git commit -m "Extract FieldData from Robot and Constants

Move the FieldConstants namespace and Robot::CheckActiveHub into a new
FieldData class, the first of the ten modeled classes to land in the tree.
Neither had any callers, so the move updates no call sites.

Fix the path where CheckActiveHub reached the end of a non-void function,
returning an indeterminate value, when the game message was R or B but the
match timer fell outside every defined window. It now returns None.

FieldData is deliberately unreferenced for now, preserving live 2026 game
logic for Robot State and Autonomous to consume during the rebuild."
```

---

## Task 3: Delete orphaned files and dead constants

**Files:**
- Delete: 22 files listed below
- Delete: `src/main/deploy/LaunchCalculator_Points.csv`
- Modify: `src/main/include/Constants.hpp` (remove `FuelConstants` and `OperatorConstants`)

**Interfaces:**
- Consumes: the tree produced by Task 2, in which nothing references any of these files.
- Produces: the final drivebase-only tree.

- [ ] **Step 1: Delete the subsystem, command and ballistics files**

```bash
git rm src/main/cpp/subsystems/Turret.cpp src/main/include/subsystems/Turret.h \
       src/main/cpp/subsystems/Turret_Shooter.cpp src/main/include/subsystems/Turret_Shooter.h \
       src/main/cpp/subsystems/TurretIntake.cpp src/main/include/subsystems/TurretIntake.h \
       src/main/cpp/subsystems/Wrist.cpp src/main/include/subsystems/Wrist.h \
       src/main/cpp/subsystems/LEDController.cpp src/main/include/subsystems/LEDController.h \
       src/main/cpp/subsystems/LED_Groups.cpp src/main/include/subsystems/LED_Groups.h \
       src/main/cpp/commands/Intake.cpp src/main/include/commands/Intake.h \
       src/main/cpp/commands/Shoot.cpp src/main/include/commands/Shoot.h \
       src/main/cpp/commands/FlattenMoonKnight.cpp src/main/include/commands/FlattenMoonKnight.h \
       src/main/cpp/commands/HalfRaiseIntake.cpp src/main/include/commands/HalfRaiseIntake.h \
       src/main/cpp/commands/BetterHalfRaise.cpp src/main/include/commands/BetterHalfRaise.h \
       src/main/cpp/utils/LaunchCalculator.cpp src/main/include/utils/LaunchCalculator.h \
       src/main/cpp/utils/BallisticsInterpolator.cpp src/main/include/utils/BallisticsInterpolator.h \
       src/main/include/utils/ballistics_rv_hub.h src/main/include/utils/ballistics_rv_gnd.h \
       src/main/deploy/LaunchCalculator_Points.csv
```

Do **not** delete `src/main/cpp/drivers/Sharp_IRDistanceSensor.cpp` or its header. It is kept deliberately unreferenced, per spec §4. Do **not** delete `utils/NetworkTableMap`, `utils/DeployFileUtils`, `utils/POIGenerator`, `utils/PoseFilter`, `subsystems/PoseEstimator`, or `commands/AutoWheelOffsets`.

- [ ] **Step 2: Remove `FuelConstants` and `OperatorConstants` from `Constants.hpp`**

Delete the entire `namespace FuelConstants { ... }` block (originally lines 25-38) and the entire `namespace OperatorConstants { ... }` block (originally lines 40-46). Both are already unreferenced anywhere in `src/`, so this is a no-op at compile time. `OperatorConstants` duplicated controller ports that are actually consumed from `DriveConstants::kDriverPort` and `kOperatorPort`.

The surviving namespaces in `Constants.hpp` are `ElectricalConstants`, `DriveConstants`, `ModuleConstants`, and `MathUtilNK`.

- [ ] **Step 3: Verify nothing references the deleted code**

```bash
grep -ri "turret\|wrist\|LEDController\|LED_Groups\|LaunchCalculator\|BallisticsInterpolator\|FuelConstants\|OperatorConstants" src/main
```
Expected: no output.

- [ ] **Step 4: Run Gate A**

Run the Gate A block from Global Constraints.
Expected: `EXIT=0`, `BUILD SUCCESSFUL`, `N actionable tasks: N executed`, error count `0`.

Confirm the Phoenix 5 and REVLib deprecation warnings are gone — their only consumers were `LED_Groups` (CANdle) and `Turret`/`Turret_Shooter`/`Wrist` (SparkMax):

```bash
grep -ci "phoenix 5 api is deprecated\|SparkBaseConfig" /tmp/gateA.log
```
Expected: `0`.

Confirm `NetworkTableMapTest` still ran and passed:

```bash
grep -c "runFrcUserProgramTest.*GoogleTestExe" /tmp/gateA.log
```
Expected: non-zero.

- [ ] **Step 5: Run Gate B**

Run the Gate B block from Global Constraints.
Expected: `EXIT=124`, startup marker `1`, crash count `0`, only `PrintLoopOverrunMessage`.

`git status` should now be clean without intervention — `LaunchCalculator_Points.csv` no longer exists and nothing writes it.

- [ ] **Step 6: Commit**

```bash
git add -A
git commit -m "Delete turret, wrist, intake and LED code

Remove the six scoring-mechanism subsystems, their five commands, the
turret-specific ballistics chain and its deploy asset, now that nothing
references them. Roughly 2,000 lines.

Drop the FuelConstants and OperatorConstants namespaces, both already
unreferenced. OperatorConstants duplicated controller ports consumed from
DriveConstants.

Keep Sharp_IRDistanceSensor unreferenced but compiling: Wrist was its only
consumer, but it is recent work the modeled Intake and Indexer will want.
Keep NetworkTableMap and DeployFileUtils, which are generic rather than
turret specific, along with the NetworkTableMapTest coverage they carry."
```

---

## Post-Plan Verification

After Task 3, confirm the end state:

```bash
export JAVA_HOME="/c/Users/Public/wpilib/2026/jdk"
./gradlew clean build --console=plain 2>&1 | grep -E "^BUILD|actionable"
ls -la build/exe/frcUserProgram/linuxathena/release/frcUserProgram
wc -l src/main/cpp/Robot.cpp src/main/include/Robot.hpp
git log --oneline -3
```

Expected: `BUILD SUCCESSFUL`, the roboRIO binary present, `Robot.cpp` around 200 lines, and three commits.

## Known Issues Left In Place

Both are recorded in spec §11 and are deliberately **not** addressed by this plan:

- `SwerveDrive::GetHeading()` has a no-return path in its NavX branch, unreachable because `m_usingPigeon` is hardcoded `true`. It is drivebase code, outside a trim.
- `FieldData::CheckActiveHub()` calls `frc::DriverStation::GetAlliance().value()` on an optional that is empty whenever no alliance is known — for example on a bench robot not connected to a driver station. That throws `std::bad_optional_access`. This is pre-existing behavior carried over verbatim, and unreachable today because nothing calls `CheckActiveHub`. It must be fixed before the first caller is added.
