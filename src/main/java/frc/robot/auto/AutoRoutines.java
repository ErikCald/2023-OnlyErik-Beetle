// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.RamseteAutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.lib2706.LTVDiffAutoBuilder;
import frc.lib.lib8727.PPLTVDiffControllerCommand;
import frc.robot.Config.AUTO;
import frc.robot.Config.DIFF;
import frc.robot.subsystems.Drive;

public final class AutoRoutines {
    private Drive drive;

    private LTVDiffAutoBuilder m_autoBuilderLTV;
    private RamseteAutoBuilder m_autoBuilderRamsete;
    private HashMap<String, Command> m_eventMap;

    public AutoRoutines(Drive drive) {
        this.drive = drive;

        setPPCommandLogging();

        m_eventMap = new HashMap<>();
        m_eventMap.put("blingBlue", new InstantCommand());

        m_autoBuilderLTV = new LTVDiffAutoBuilder(
            drive::getPose, 
            drive::resetPose, 
            DIFF.CONTROLLER, 
            DIFF.FEEDFORWARD, 
            DIFF.KINEMATICS, 
            drive::getWheelSpeeds, 
            drive::setWheelVoltages, 
            m_eventMap, 
            false, 
            drive);

    }

    /** Example static factory for an autonomous command. */
    public CommandBase doNothing() {
        return null;
    }

    public CommandBase drivePath(String pathName) {
        List<PathPlannerTrajectory> traj = PathPlanner.loadPathGroup(pathName, AUTO.VEL, AUTO.ACCEL);
        return m_autoBuilderLTV.followPathGroup(traj);
    }



    private void setPPCommandLogging() {
        PPLTVDiffControllerCommand.setControllerDisabled(AUTO.DISABLE_FEEDBACK);

        PPLTVDiffControllerCommand.setLoggingCallbacks(
            null, 
            null, 
            null, 
            this::defaultLogError, 
            this::logMeasuredSpeeds, 
            this::logTargetSpeeds
        );
    }

    private void defaultLogError(Translation2d translationError, Rotation2d rotationError) {
        SmartDashboard.putNumber("PPLTVDiffControllerCommand/xErrorMeters", translationError.getX());
        SmartDashboard.putNumber("PPLTVDiffControllerCommand/yErrorMeters", translationError.getY());
        SmartDashboard.putNumber(
            "PPLTVDiffControllerCommand/rotationErrorDegrees", rotationError.getDegrees());
    }

    private void logMeasuredSpeeds(DifferentialDriveWheelSpeeds diffSpeeds) {
        ChassisSpeeds speeds = DIFF.KINEMATICS.toChassisSpeeds(diffSpeeds);
        SmartDashboard.putNumber("PPLTVDiffControllerCommand/measuredSpeed", speeds.vxMetersPerSecond);
        SmartDashboard.putNumber("PPLTVDiffControllerCommand/measuredAngular", speeds.omegaRadiansPerSecond);
    }

    private void logTargetSpeeds(ChassisSpeeds speeds) {
        SmartDashboard.putNumber("PPLTVDiffControllerCommand/targetSpeed", speeds.vxMetersPerSecond);
        SmartDashboard.putNumber("PPLTVDiffControllerCommand/targetAngular", speeds.omegaRadiansPerSecond);
    }
     
}
