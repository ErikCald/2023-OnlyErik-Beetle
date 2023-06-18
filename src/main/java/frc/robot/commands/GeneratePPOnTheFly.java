// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.BaseAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class GeneratePPOnTheFly extends CommandBase {
    private BaseAutoBuilder m_autoBuilder;
    private DriveSubsystem m_drive;
    private double m_maxVel;
    private double m_maxAccel;
    private List<PathPoint> m_pathPoints;

    private CommandBase m_activeCommand;
    
    /**
     * Generates a PathPlanner command to start from the robots current location and speed, and goes through the finalPathPoints.
     * 
     * If you want to schedule commands while moving, recommend creating a seperate PathPlanner command to run right after this command.
     *      (and putting a override velocity in the last point of finalPathPoints)
     * 
     * @param autoBuilder
     * @param drive
     * @param maxVel
     * @param maxAccel
     * @param finalPathPoints
     */
    public GeneratePPOnTheFly(BaseAutoBuilder autoBuilder, DriveSubsystem drive, double maxVel, double maxAccel, List<PathPoint> finalPathPoints) {
        m_autoBuilder = autoBuilder;
        m_drive = drive;
        m_maxVel = maxVel;
        m_maxAccel = maxAccel;
        m_pathPoints = finalPathPoints;

        m_pathPoints.add(0, null);

        addRequirements(drive);
    }


    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Pose2d pose = m_drive.getPose();
        ChassisSpeeds speeds = m_drive.getSpeeds();

        m_pathPoints.set(0, new PathPoint(
            pose.getTranslation(), 
            new Rotation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond), 
            Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond)
        ));

        PathPlannerTrajectory traj = PathPlanner.generatePath(
            new PathConstraints(m_maxVel, m_maxAccel), 
            m_pathPoints
        );

        m_activeCommand = m_autoBuilder.followPath(traj);

        m_activeCommand.initialize();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_activeCommand.execute();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_activeCommand.end(interrupted);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_activeCommand.isFinished();
    }
}
