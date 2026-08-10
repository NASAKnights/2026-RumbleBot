// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "drivers/Sharp_IRDistanceSensor.h"

#define VOLTAGE_POINT_1 2.3
#define INV_DISTANCE_POINT_1 (1/10)

#define VOLTAGE_POINT_2 0.4
#define INV_DISTANCE_POINT_2 (1/80)

#define TRANSFER_SLOPE ((VOLTAGE_POINT_2 - VOLTAGE_POINT_1) / (INV_DISTANCE_POINT_2 - INV_DISTANCE_POINT_1))
#define TRANSFER_INTERCEPT (((INV_DISTANCE_POINT_2 * VOLTAGE_POINT_1) - (INV_DISTANCE_POINT_1 * VOLTAGE_POINT_2)) \
    / (INV_DISTANCE_POINT_2 - INV_DISTANCE_POINT_1))

Sharp_IRDistanceSensor::Sharp_IRDistanceSensor(int channel) : 
    m_input(frc::AnalogInput(channel))
{
    
}

units::centimeter_t Sharp_IRDistanceSensor::GetDistance()
{
    //get voltage
    auto sensor_voltage = m_input.GetVoltage();
    //translate voltage to distance
    
    //return value
}
