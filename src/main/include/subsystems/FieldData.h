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
