package frc.robot.auto;

import java.util.Map;

import com.pathplanner.lib.auto.BaseAutoBuilder;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.RamseteAutoBuilder;
import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.lib2706.LTVDiffAutoBuilder;
import frc.lib.lib8727.PPLTVDiffControllerCommand;
import frc.robot.Config.AUTO;
import frc.robot.Config.DIFF;
import frc.robot.subsystems.DriveSubsystem;

public class CreateAutoBuilder {

    /**
     * Creates a PathPlanner auto builder and returns the instance.
     * 
     * @param drive The drive subsystem
     * @param eventMap A PathPlanner event map.
     * @return An auto builder.
     */
    public static BaseAutoBuilder createBuilder(DriveSubsystem drive, Map<String, Command> eventMap) {
        setPPCommandLogging(drive);
        if (AUTO.USE_RAMSETE_NOT_LTV) {
            return new RamseteAutoBuilder(
                drive::getPose, 
                drive::resetPose, 
                DIFF.RAMSETE_CONTROLLER, 
                DIFF.KINEMATICS, 
                DIFF.WHEEL_FEEDFORWARD, 
                drive::getWheelSpeeds, 
                disablePID(DIFF.WHEEL_PID, AUTO.DISABLE_FEEDBACK), 
                drive::setWheelVoltages, 
                eventMap, 
                false, 
                drive
            );
        } else {
            return new LTVDiffAutoBuilder(
                drive::getPose, 
                drive::resetPose, 
                DIFF.LTV_CONTROLLER, 
                DIFF.DIFF_FEEDFORWARD, 
                DIFF.KINEMATICS, 
                drive::getWheelSpeeds, 
                drive::setWheelVoltages, 
                eventMap, 
                false, 
                drive
            ); 
        }
    }

    private static void setPPCommandLogging(DriveSubsystem drive) {
        if (AUTO.USE_RAMSETE_NOT_LTV) {
            PPRamseteCommand.setLoggingCallbacks(
                drive::setField2dTrajectory, 
                null, 
                CreateAutoBuilder::logRamseteTargetSpeeds, 
                CreateAutoBuilder::defaultLogError
            );
        } else {
            PPLTVDiffControllerCommand.setControllerDisabled(AUTO.DISABLE_FEEDBACK);

            PPLTVDiffControllerCommand.setLoggingCallbacks(
                drive::setField2dTrajectory, 
                null, 
                null, 
                CreateAutoBuilder::defaultLogError, 
                CreateAutoBuilder::logMeasuredSpeeds, 
                CreateAutoBuilder::logTargetSpeeds
            );
        }
    }

    private static String getTable() {
        if (AUTO.USE_RAMSETE_NOT_LTV) {
            return "PPRamseteCommand";
        } else {
            return "PPLTVDiffControllerCommand";
        }
    }

    private static void defaultLogError(Translation2d translationError, Rotation2d rotationError) {
        SmartDashboard.putNumber(getTable()+"/xErrorMeters", translationError.getX());
        SmartDashboard.putNumber(getTable()+"/yErrorMeters", translationError.getY());
        SmartDashboard.putNumber(getTable()+
            "/rotationErrorDegrees", rotationError.getDegrees());
    }

    private static void logMeasuredSpeeds(DifferentialDriveWheelSpeeds diffSpeeds) {
        ChassisSpeeds speeds = DIFF.KINEMATICS.toChassisSpeeds(diffSpeeds);
        SmartDashboard.putNumber(getTable()+"/measuredSpeed", speeds.vxMetersPerSecond);
        SmartDashboard.putNumber(getTable()+"/measuredAngular", speeds.omegaRadiansPerSecond);
    }

    private static void logTargetSpeeds(ChassisSpeeds speeds) {
        SmartDashboard.putNumber(getTable()+"/targetSpeed", speeds.vxMetersPerSecond);
        SmartDashboard.putNumber(getTable()+"/targetAngular", speeds.omegaRadiansPerSecond);
    }

    private static void logRamseteTargetSpeeds(ChassisSpeeds speeds) {
        DifferentialDriveWheelSpeeds diffSpeeds = DIFF.KINEMATICS.toWheelSpeeds(speeds);
        SmartDashboard.putNumber(getTable()+"targetLeftSpeed", diffSpeeds.leftMetersPerSecond);
        SmartDashboard.putNumber(getTable()+"targetRightSpeed", diffSpeeds.rightMetersPerSecond);
    }

    private static PIDConstants disablePID(PIDConstants pid, boolean shouldDisable) {
        if (shouldDisable) {
            return new PIDConstants(0, 0, 0);
        } else {
            return pid;
        }
    }
}
