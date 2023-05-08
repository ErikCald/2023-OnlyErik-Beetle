// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Config.JOYSTICK;
import frc.robot.subsystems.DriveSubsystem;

public class ArcadeDrive extends CommandBase {
    DriveSubsystem m_drive;
    Supplier<Double> m_forward;
    Supplier<Double> m_steering;

    /**
     * Use the other constructor. Controls a Differential Drive robot in teleop.
     * 
     * This constructor will not handle the deadband or inverting the forward axis.
     * 
     * @param joystickForward Supplier to get the desired forward movement, range [-1, 1]
     * @param joystickSteering Supplier to get the desired steering movement, range [-1, 1]
     * @param subsystem The DriveSubsystem
     */
    public ArcadeDrive(Supplier<Double> joystickForward, Supplier<Double> joystickSteering, DriveSubsystem drive) {
        m_forward = joystickForward;
        m_steering = joystickSteering;
        m_drive = drive;

        addRequirements(drive);
    }

    /**
     * Prefered constructor. Controls a Differential Drive robot in teleop.
     * 
     * Inverts the forward axis.
     * Applys a deadband (value from Config.java)
     * 
     * @param driverStick Joystick of the driver's XboxController (or other controllers)
     * @param forwardAxis id of the axis for forwards
     * @param steeringAxis id of the axis for steering
     * @param subsystem The DriveSubsystem
     */
    public ArcadeDrive(CommandXboxController driverStick, int forwardAxis, int steeringAxis, DriveSubsystem drive) {
        this(
            () -> -1 * MathUtil.applyDeadband(driverStick.getRawAxis(forwardAxis), JOYSTICK.DRIVER_JOYSTICK_DEADBAND),
            () -> -1 * MathUtil.applyDeadband(driverStick.getRawAxis(steeringAxis), JOYSTICK.DRIVER_JOYSTICK_DEADBAND),
            drive
        );
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // m_drive.setNeutralMode(DIFF.TELEOP_NEUTRALMODE);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_drive.arcadeDrive(m_forward.get(), m_steering.get());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
