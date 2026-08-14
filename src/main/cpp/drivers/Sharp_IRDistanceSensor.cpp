// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "drivers/Sharp_IRDistanceSensor.h"

#define VOLTAGE_POINT_1 2.3
#define INV_DISTANCE_POINT_1 (1.0/10.0)

#define VOLTAGE_POINT_2 0.4
#define INV_DISTANCE_POINT_2 (1.0/80.0)

// (y2 - y1) / (x2 - x1)
#define TRANSFER_SLOPE ((VOLTAGE_POINT_2 - VOLTAGE_POINT_1) / (INV_DISTANCE_POINT_2 - INV_DISTANCE_POINT_1))
#define TRANSFER_INTERCEPT (((INV_DISTANCE_POINT_2 * VOLTAGE_POINT_1) - (INV_DISTANCE_POINT_1 * VOLTAGE_POINT_2)) \
    / (INV_DISTANCE_POINT_2 - INV_DISTANCE_POINT_1))

const units::centimeter_t Sharp_IRDistanceSensor::max_distance = units::centimeter_t{80};
const units::centimeter_t Sharp_IRDistanceSensor::min_distance = units::centimeter_t{10};

Sharp_IRDistanceSensor::Sharp_IRDistanceSensor(int channel) : 
    m_input(frc::AnalogInput(channel))
{
    
}

units::centimeter_t Sharp_IRDistanceSensor::GetDistance()
{
    //get voltage
    auto sensor_voltage = m_input.GetVoltage();
    //translate voltage to distance
    // 2. Prevent division by zero or negative distance if voltage drops below the intercept
    if (sensor_voltage < VOLTAGE_POINT_2) {
        return units::centimeter_t{-1.0}; // Max useful range for GP2Y0A21YK0F
    }

    // 3. Solve V = m * (1/d) + b  ==>  d = m / (V - b)
    double distance_cm = 1 / (TRANSFER_SLOPE / (sensor_voltage - TRANSFER_INTERCEPT));

    // 4. Clamp within valid sensor operational range (10cm - 80cm)
    distance_cm = std::clamp(distance_cm, double{Sharp_IRDistanceSensor::min_distance}, 
        double{Sharp_IRDistanceSensor::max_distance});

    return units::centimeter_t{distance_cm};
    //return value
}
