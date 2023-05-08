// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.lib8727.PPLTVDiffControllerCommand;
import frc.robot.Config.JOYSTICK;
import frc.robot.auto.AutoRoutines;
import frc.robot.commands.ArcadeDrive;
import frc.robot.subsystems.DriveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    DriveSubsystem drive = new DriveSubsystem();

    AutoRoutines autoRoutines = new AutoRoutines(drive);

    // Joysticks
    private final CommandXboxController driver = new CommandXboxController(JOYSTICK.DRIVER_JOYSTICK_PORT);
    private final CommandXboxController operator = new CommandXboxController(JOYSTICK.OPERATOR_JOYSTICK_PORT);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the trigger bindings
        configureBindings();
    }

    /**
     * Use this method to define your trigger->command mappings.
     */
    private void configureBindings() {
        drive.setDefaultCommand(new ArcadeDrive(driver, 1, 2, drive));// XboxController.Axis.kLeftY.value, XboxController.Axis.kRightX.value, drive));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        // return autoRoutines.doNothing();

        // return autoRoutines.drivePath("driveForward");
        // return autoRoutines.drivePath("curve");
        // return autoRoutines.drivePath("sCurve");
        return autoRoutines.drivePath("fancy");
    }
}
