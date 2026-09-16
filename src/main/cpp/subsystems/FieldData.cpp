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
