// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.lib2706;

import frc.robot.Config.DIFF;

/**
 * Class to convert between metric units and CTRE units.
 */
public class CTREUnits {
    private static final double WHEEL_DIAMETER = DIFF.WHEEL_DIAMETER;
    private static final double TICKS_PER_REVOLUTION = 4096;
    /**
     * Converting Talon ticks to meters
     * 
     * Unit Conversion Method
     */
    public static double talonPositionToMeters(double talonPosisiton) {
        double result = talonPosisiton;
        double circumference = Math.PI * 0.1524; // 6 inches = 0.1524 meters;
        double metersPerTick = circumference / TICKS_PER_REVOLUTION;
        result *= metersPerTick;
        return result;  
    }

    /**
     * Converting m/s to talon ticks/100ms
     *  
     * Unit Conversion Method
     */
    public static double metersPerSecondToTalonVelocity(double metersPerSecond) {
        return metersToTalonPosistion(metersPerSecond * 0.1); // Converting meters per second to meters per 100ms
    }

    /**
     * Converting meters to talon ticks
     * 
     * Unit Conversion Method
     */
    public static double metersToTalonPosistion(double meters) {
        double result = meters;
        double circumference = Math.PI * 0.1524; // 6 inches = 0.1524 meters; // Pi*Diameter
        double ticksPerMeter = TICKS_PER_REVOLUTION / circumference; // Ticks per revolution / circumference
        result = result * ticksPerMeter; // Meter * ticks in 1 meter
        return result;
    }

    /**
     * Converting Talon ticks to m/s
     * 
     * Unit Conversion Method
     */
    public static double talonPosistionToMeters(double talonPosisiton) {
        double result = talonPosisiton;
        double circumference = Math.PI * 0.1524; // 6 inches = 0.1524 meters;
        double metersPerTick = circumference / TICKS_PER_REVOLUTION;
        result *= metersPerTick;
        return result;

    }

    /**
     * Converting talon ticks/100ms to m/s
     * 
     * Unit Conversion Method
     */
    public static double talonVelocityToMetersPerSecond(double talonVelocity) {
        return talonPosistionToMeters(talonVelocity * 10); // Convert ticks/100ms to ticks/sec then convert ticks/sec to m/s
    }
}