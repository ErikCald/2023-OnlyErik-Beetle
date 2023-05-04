// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.DifferentialDriveFeedforward;
import edu.wpi.first.math.controller.LTVDifferentialDriveController;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;

/**
 * The Config class provides a convenient place for robot-wide numerical or
 * boolean constants. This class should not be used for any other purpose. All
 * constants should be declared globally (i.e. public static). Do not put
 * anything functional in this class.
 */
public final class Config {
    public static class GENERAL {
        public static final double RIO_CYCLE_TIME = 0.02;
    }

    public static class JOYSTICK {
        public static final int DRIVER_JOYSTICK_PORT = 0;
        public static final int OPERATOR_JOYSTICK_PORT = 1;

        public static final double DRIVER_JOYSTICK_DEADBAND = 0.1;
    }

    public static class CANID {
        public static final int DIFF_LEFT = 2;
        public static final int DIFF_RIGHT = 1;

        public static final int PIGEON = 27;
    }

    public static class DIFF {
        public static final boolean INVERT_LEFT_MOTOR = false;
        public static final boolean INVERT_RIGHT_MOTOR = true;
        public static final boolean INVERT_LEFT_ENCODER = false;
        public static final boolean INVERT_RIGHT_ENCODER = false;

        public static final double WHEEL_DIAMETER = 0.1016; // 4 inches = 0.1016 meters

        public static final double TRACKWIDTH = 0.3136;
        public static final DifferentialDriveKinematics KINEMATICS = new DifferentialDriveKinematics(TRACKWIDTH);


        public static final double kV_LINEAR = 2.5; // Values copied from 6498
        public static final double kA_LINEAR = 1;
        public static final double kV_ANGULAR = 0.37;
        public static final double KA_ANGULAR = 0.12;

        public static final DifferentialDriveFeedforward FEEDFORWARD = new DifferentialDriveFeedforward (
            kV_LINEAR, 
            kA_LINEAR, 
            kV_ANGULAR, 
            KA_ANGULAR,
            TRACKWIDTH
        );
        
        public static final LTVDifferentialDriveController CONTROLLER = new LTVDifferentialDriveController(
            LinearSystemId.identifyDrivetrainSystem(kV_LINEAR, kA_LINEAR, kV_ANGULAR, KA_ANGULAR, TRACKWIDTH),
            TRACKWIDTH,
            VecBuilder.fill(// Used for vision rotating align from another team: VecBuilder.fill(0.001, 0.001, 0.001, 0.5, 1)
                0.0625, // X pos meters
                0.125,  // Y pos meters
                1.5,    // Heading radians
                0.95,   // Left velocity meters per second
                0.95),  // Right velocity meters per second
            VecBuilder.fill(12.0, 12.0), // Volts
            GENERAL.RIO_CYCLE_TIME);
    }

    public static class DIFF_SIMULATION {
        public static final double MOTOR_GEAR_RATIO = 10.71;
        public static final double MASS_KG = Units.lbsToKilograms(100);
        // MOI estimation -- note that I = mr² for point masses
        private static final double batteryMoi = 12.5 / 2.2 * Math.pow(Units.inchesToMeters(10), 2);
        private static final double gearboxMoi = (2.8 /* CIM motor */ * 2 / 2.2 + 2.0 /* Toughbox Mini- ish */)
                                                     * Math.pow(Units.inchesToMeters(26.0 / 2.0), 2);

        /* Simulation model of the drivetrain */
        public static final DifferentialDrivetrainSim SIMULATION = new DifferentialDrivetrainSim(
            DCMotor.getCIM(1), //1 CIM on each side of the drivetrain.
            MOTOR_GEAR_RATIO,
            batteryMoi + gearboxMoi, // MOI estimation (alternatively, can be from a CAD model).
            MASS_KG, //Mass of the robot
            DIFF.WHEEL_DIAMETER,  //Robot uses 3" radius (6" diameter) wheels.
            DIFF.TRACKWIDTH,                    //Distance between wheels is _ meters.
            null//VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005)
        );
    }

    public static class AUTO {
        public static final boolean DISABLE_FEEDBACK = false;
        public static final double VEL = 3;
        public static final double ACCEL = 5;
    }
}
