package frc.lib.lib2706;

import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.BaseAutoBuilder;

import edu.wpi.first.math.controller.DifferentialDriveFeedforward;
import edu.wpi.first.math.controller.DifferentialDriveWheelVoltages;
import edu.wpi.first.math.controller.LTVDifferentialDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.lib.lib8727.PPLTVDiffControllerCommand;

public class LTVDiffAutoBuilder extends BaseAutoBuilder {
  private final LTVDifferentialDriveController controller;
  private final DifferentialDriveFeedforward feedforward;
  private final DifferentialDriveKinematics kinematics;
  private final Supplier<DifferentialDriveWheelSpeeds> speedsSupplier;
  private final Consumer<DifferentialDriveWheelVoltages> outputVolts;
  private final Subsystem[] driveRequirements;

  /**
   * Create an auto builder that will create command groups that will handle path following and
   * triggering events.
   *
   * <p>This auto builder will use PPRamseteCommand to follow paths.
   *
   * @param poseSupplier A function that supplies the robot pose - use one of the odometry classes
   *     to provide this.
   * @param resetPose A consumer that accepts a Pose2d to reset robot odometry. This will typically
   *     be called once at the beginning of an auto.
   * @param controller The RAMSETE controller used to follow the trajectory.
   * @param kinematics The kinematics for the robot drivetrain.
   * @param feedforward The feedforward to use for the drive.
   * @param speedsSupplier A function that supplies the speeds of the left and right sides of the
   *     robot drive.
   * @param driveConstants PIDConstants for each side of the drive train
   * @param outputVolts Output consumer that accepts left and right voltages
   * @param eventMap Map of event marker names to the commands that should run when reaching that
   *     marker.
   * @param useAllianceColor Should the path states be automatically transformed based on alliance
   *     color? In order for this to work properly, you MUST create your path on the blue side of
   *     the field.
   * @param driveRequirements The subsystems that the path following commands should require.
   *     Usually just a Drive subsystem.
   */
  public LTVDiffAutoBuilder(
      Supplier<Pose2d> poseSupplier,
      Consumer<Pose2d> resetPose,
      LTVDifferentialDriveController controller,
      DifferentialDriveFeedforward feedforward,
      DifferentialDriveKinematics kinematics,
      Supplier<DifferentialDriveWheelSpeeds> speedsSupplier,
      Consumer<DifferentialDriveWheelVoltages> outputVolts,
      Map<String, Command> eventMap,
      boolean useAllianceColor,
      Subsystem... driveRequirements) {
    super(poseSupplier, resetPose, eventMap, DrivetrainType.STANDARD, useAllianceColor);

    this.controller = controller;
    this.feedforward = feedforward;
    this.kinematics = kinematics;
    this.speedsSupplier = speedsSupplier;
    this.outputVolts = outputVolts;
    this.driveRequirements = driveRequirements;
  }

  @Override
  public CommandBase followPath(PathPlannerTrajectory trajectory) {
    return new PPLTVDiffControllerCommand(
      trajectory, 
      poseSupplier, 
      resetPose, 
      controller, 
      feedforward, 
      kinematics, 
      speedsSupplier, 
      outputVolts, 
      useAllianceColor, 
      driveRequirements);
  }
}
