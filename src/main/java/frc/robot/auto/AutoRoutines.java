// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.BaseAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Config.AUTO;
import frc.robot.subsystems.DriveSubsystem;

public final class AutoRoutines {
    private DriveSubsystem drive;

    private BaseAutoBuilder m_autoBuilder;
    private HashMap<String, Command> m_eventMap;

    public AutoRoutines(DriveSubsystem drive) {
        this.drive = drive;

        m_autoBuilder = CreateAutoBuilder.createBuilder(drive, m_eventMap);

        m_eventMap = new HashMap<>();
        m_eventMap.put("blingBlue", new InstantCommand());

    }
    
    /** Example static factory for an autonomous command. */
    public CommandBase doNothing() {
        return null;
    }

    public CommandBase drivePath(String pathName) {
        List<PathPlannerTrajectory> traj = PathPlanner.loadPathGroup(pathName, AUTO.VEL, AUTO.ACCEL);
        return m_autoBuilder.resetPose(traj.get(0)).andThen(m_autoBuilder.followPathGroup(traj));
    }
}
