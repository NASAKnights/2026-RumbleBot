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
                    }, {&m_swerveDrive})))
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
