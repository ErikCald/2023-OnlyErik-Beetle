// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.BasePigeonSimCollection;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.controller.DifferentialDriveWheelVoltages;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.lib2706.CTREUnits;
import frc.robot.Config.CANID;
import frc.robot.Config.DIFF;
import frc.robot.Config.DIFF_SIMULATION;

public class DriveSubsystem extends SubsystemBase {
    WPI_TalonSRX m_leftMotor, m_rightMotor;
    PigeonIMU m_pigeon;

    DifferentialDrive m_teleopDrive;
    DifferentialDrivePoseEstimator m_poseEstimator;

    Field2d m_field = new Field2d();
    
    TalonSRXSimCollection m_simLeftMotor, m_simRightMotor;
    BasePigeonSimCollection m_simPigeon;

    DoublePublisher pubLeftVel, pubRightVel;

    /** Creates a new DifferentialDrive. */
    public DriveSubsystem() {
        m_leftMotor = new WPI_TalonSRX(CANID.DIFF_LEFT);
        m_rightMotor = new WPI_TalonSRX(CANID.DIFF_RIGHT);

        m_leftMotor.configFactoryDefault();
        m_rightMotor.configFactoryDefault();

        m_leftMotor.setInverted(DIFF.INVERT_LEFT_MOTOR);
        m_rightMotor.setInverted(DIFF.INVERT_RIGHT_MOTOR);

        m_leftMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        m_rightMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

        // Invert encoders if boolean is true
        m_leftMotor.setSensorPhase(DIFF.INVERT_LEFT_ENCODER);
        m_rightMotor.setSensorPhase(DIFF.INVERT_RIGHT_ENCODER);

        m_teleopDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);

        m_pigeon = new PigeonIMU(CANID.PIGEON);

        m_simLeftMotor = m_leftMotor.getSimCollection();
        m_simRightMotor = m_rightMotor.getSimCollection();
        m_simPigeon = m_pigeon.getSimCollection();

        m_poseEstimator = new DifferentialDrivePoseEstimator(
            DIFF.KINEMATICS, 
            getGyroYaw(),
            getLeftDistance(),
            getRightDistance(),
            new Pose2d()); 

        SmartDashboard.putData("Field", m_field);

        createNT();
    }

    private void createNT() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("Drive");
        pubLeftVel = table.getDoubleTopic("MeasuredLeftVelocity").publish(PubSubOption.periodic(0.02));
        pubRightVel = table.getDoubleTopic("MeasuredRightVelocity").publish(PubSubOption.periodic(0.02));
        
    }

    @Override
    public void periodic() {
        Pose2d newPose = m_poseEstimator.update(
            getGyroYaw(), 
            getLeftDistance(), 
            getRightDistance());

        m_field.setRobotPose(newPose);
    }

    public void arcadeDrive(double leftSpeed, double rightSpeed) {
        m_teleopDrive.arcadeDrive(leftSpeed, rightSpeed);
    }
    
    /**
     * Get the fused heading from the pigeon
     * 
     * @return Heading of the robot in degrees
     */
    private Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(m_pigeon.getFusedHeading());
    }

    /**
     * Get the encoder data in meters
     * 
     * @return Distance of the left encoder in meters
     */
    private double getLeftDistance() {
        return CTREUnits.talonPosistionToMeters(m_leftMotor.getSelectedSensorPosition());
    }

    /**
     * Get the encoder data in meters
     * 
     * @return Distance of the right encoder in meters
     */
    private double getRightDistance() {
        return CTREUnits.talonPosistionToMeters(m_rightMotor.getSelectedSensorPosition());
    }

    private double getLeftVelocity() {
        return CTREUnits.talonVelocityToMetersPerSecond(m_leftMotor.getSelectedSensorVelocity());
    }

    private double getRightVelocity() {
        return CTREUnits.talonVelocityToMetersPerSecond(m_rightMotor.getSelectedSensorVelocity());
    }
    
    public Pose2d getPose() {
        return m_poseEstimator.getEstimatedPosition();
    }

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    public void resetPose(Pose2d pose) {
        m_poseEstimator.resetPosition(
            getGyroYaw(), 
            getLeftDistance(), 
            getRightDistance(), 
            pose);
    }

    /**
     * Get the measured velocity of the left and right 
     * wheels in meters per second.
     * 
     * @return The speeds of the wheels in m/s.
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        double leftSpeed = getLeftVelocity();
        double rightSpeed = getRightVelocity();

        pubLeftVel.accept(leftSpeed);
        pubRightVel.accept(rightSpeed);

        return new DifferentialDriveWheelSpeeds(
            leftSpeed,
            rightSpeed
        );
    }

    public void setWheelVoltages(double leftVoltage, double rightVoltage) {
        m_leftMotor.setVoltage(leftVoltage);
        m_rightMotor.setVoltage(rightVoltage);
    }

    public void setWheelVoltages(DifferentialDriveWheelVoltages voltages) {
        setWheelVoltages(voltages.left, voltages.right);
    }

    public void stopMotors() {
        m_leftMotor.stopMotor();
        m_rightMotor.stopMotor();
    }

    public void setField2dTrajectory(PathPlannerTrajectory traj) {
        m_field.getObject("traj").setTrajectory((Trajectory) traj);
    }


    @Override
    public void simulationPeriodic() {
        /* Pass the robot battery voltage to the simulated Talon SRXs */
        m_simLeftMotor.setBusVoltage(RobotController.getBatteryVoltage());
        m_simRightMotor.setBusVoltage(RobotController.getBatteryVoltage());

        /*
         * CTRE simulation is low-level, so SimCollection inputs
         * and outputs are not affected by SetInverted(). Only
         * the regular user-level API calls are affected.
         *
         * WPILib expects +V to be forward.
         * Positive motor output lead voltage is ccw. We observe
         * on our physical robot that this is reverse for the
         * right motor, so negate it.
         *
         * We are hard-coding the negation of the values instead of
         * using getInverted() so we can catch a possible bug in the
         * robot code where the wrong value is passed to setInverted().
         */
        DIFF_SIMULATION.SIMULATION.setInputs(m_simLeftMotor.getMotorOutputLeadVoltage(),
                            -m_simRightMotor.getMotorOutputLeadVoltage());

        /*
         * Advance the model by 20 ms. Note that if you are running this
         * subsystem in a separate thread or have changed the nominal
         * timestep of TimedRobot, this value needs to match it.
         */
        DIFF_SIMULATION.SIMULATION.update(0.02);

        /*
         * Update all of our sensors.
         *
         * Since WPILib's simulation class is assuming +V is forward,
         * but -V is forward for the right motor, we need to negate the
         * position reported by the simulation class. Basically, we
         * negated the input, so we need to negate the output.
         *
         * We also observe on our physical robot that a positive voltage
         * across the output leads results in a positive sensor velocity
         * for both the left and right motors, so we do not need to negate
         * the output any further.
         * If we had observed that a positive voltage results in a negative
         * sensor velocity, we would need to negate the output once more.
         */
        m_simLeftMotor.setQuadratureRawPosition(
            (int) CTREUnits.metersToTalonPosistion(
                DIFF_SIMULATION.SIMULATION.getLeftPositionMeters()));

        m_simLeftMotor.setQuadratureVelocity(
            (int) CTREUnits.metersPerSecondToTalonVelocity(
                DIFF_SIMULATION.SIMULATION.getLeftVelocityMetersPerSecond()));

        m_simRightMotor.setQuadratureRawPosition(
            (int) CTREUnits.metersToTalonPosistion(
                -DIFF_SIMULATION.SIMULATION.getRightPositionMeters()));

        m_simRightMotor.setQuadratureVelocity(
            (int) CTREUnits.metersPerSecondToTalonVelocity(
                -DIFF_SIMULATION.SIMULATION.getRightVelocityMetersPerSecond()));


        m_simPigeon.setRawHeading(DIFF_SIMULATION.SIMULATION.getHeading().getDegrees());

    }
}
