package frc.lib.lib8727;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.math.controller.DifferentialDriveFeedforward;
import edu.wpi.first.math.controller.DifferentialDriveWheelVoltages;
import edu.wpi.first.math.controller.LTVDifferentialDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class PPLTVDiffControllerCommand extends CommandBase {
  private final Timer timer = new Timer();
  private final PathPlannerTrajectory trajectory;
  private final Supplier<Pose2d> poseSupplier;
  private final Consumer<Pose2d> poseReset;
  private final LTVDifferentialDriveController controller;
  private final DifferentialDriveFeedforward feedforward;
  private final DifferentialDriveKinematics kinematics;
  private final Supplier<DifferentialDriveWheelSpeeds> speedsSupplier;
  private final Consumer<DifferentialDriveWheelVoltages> output;
  private final boolean useAllianceColor;

  private PathPlannerTrajectory transformedTrajectory;

  private static Consumer<PathPlannerTrajectory> logActiveTrajectory = null;
  private static Consumer<Pose2d> logTargetPose = null;
  private static Consumer<DifferentialDriveWheelVoltages> logSetpoint = null;
  private static BiConsumer<Translation2d, Rotation2d> logError =
      PPLTVDiffControllerCommand::defaultLogError;

  private static Consumer<DifferentialDriveWheelSpeeds> logMeasuredSpeeds = null;
  private static Consumer<ChassisSpeeds> logTargetSpeeds = null;

  private static boolean disableController = false;

  /**
   * Constructs a new PPRamseteCommand that, when executed, will follow the provided trajectory. PID
   * control and feedforward are handled internally, and outputs are scaled -12 to 12 representing
   * units of volts.
   *
   * <p>Note: The controller will *not* set the outputVolts to zero upon completion of the path -
   * this is left to the user, since it is not appropriate for paths with nonstationary endstates.
   *
   * @param trajectory The trajectory to follow.
   * @param poseSupplier A function that supplies the robot pose - use one of the odometry classes
   *     to provide this.
   * @param controller The RAMSETE controller used to follow the trajectory.
   * @param speedsSupplier A function that supplies the speeds of the left and right sides of the
   *     robot drive.
   * @param outputVolts A function that consumes the computed left and right outputs (in volts) for
   *     the robot drive.
   * @param useAllianceColor Should the path states be automatically transformed based on alliance
   *     color? In order for this to work properly, you MUST create your path on the blue side of
   *     the field.
   * @param requirements The subsystems to require.
   */
  public PPLTVDiffControllerCommand(
      PathPlannerTrajectory trajectory,
      Supplier<Pose2d> poseSupplier,
      Consumer<Pose2d> poseReset,
      LTVDifferentialDriveController controller,
      DifferentialDriveFeedforward feedforward,
      DifferentialDriveKinematics kinematics,
      Supplier<DifferentialDriveWheelSpeeds> speedsSupplier,
      Consumer<DifferentialDriveWheelVoltages> outputVolts,
      boolean useAllianceColor,
      Subsystem... requirements) {
    this.trajectory = trajectory;
    this.poseSupplier = poseSupplier;
    this.poseReset = poseReset;
    this.controller = controller;
    this.feedforward = feedforward;
    this.kinematics = kinematics;
    this.speedsSupplier = speedsSupplier;
    this.output = outputVolts;
    this.useAllianceColor = useAllianceColor;

    addRequirements(requirements);

    if (useAllianceColor && trajectory.fromGUI && trajectory.getInitialPose().getX() > 8.27) {
      DriverStation.reportWarning(
          "You have constructed a path following command that will automatically transform path states depending"
              + " on the alliance color, however, it appears this path was created on the red side of the field"
              + " instead of the blue side. This is likely an error.",
          false);
    }
  }

  /**
   * Constructs a new PPRamseteCommand that, when executed, will follow the provided trajectory. PID
   * control and feedforward are handled internally, and outputs are scaled -12 to 12 representing
   * units of volts.
   *
   * <p>Note: The controller will *not* set the outputVolts to zero upon completion of the path -
   * this is left to the user, since it is not appropriate for paths with nonstationary endstates.
   *
   * @param trajectory The trajectory to follow.
   * @param poseSupplier A function that supplies the robot pose - use one of the odometry classes
   *     to provide this.
   * @param controller The RAMSETE controller used to follow the trajectory.
   * @param feedforward The feedforward to use for the drive.
   * @param speedsSupplier A function that supplies the speeds of the left and right sides of the
   *     robot drive.
   * @param outputVolts A function that consumes the computed left and right outputs (in volts) for
   *     the robot drive.
   * @param requirements The subsystems to require.
   */
  public PPLTVDiffControllerCommand(
      PathPlannerTrajectory trajectory,
      Supplier<Pose2d> poseSupplier,
      Consumer<Pose2d> poseReset,
      LTVDifferentialDriveController controller,
      DifferentialDriveFeedforward feedforward,
      DifferentialDriveKinematics kinematics,
      Supplier<DifferentialDriveWheelSpeeds> speedsSupplier,
      Consumer<DifferentialDriveWheelVoltages> outputVolts,
      Subsystem... requirements) {
    this(
        trajectory,
        poseSupplier,
        poseReset,
        controller,
        feedforward,
        kinematics,
        speedsSupplier,
        outputVolts,
        false,
        requirements);
  }

  @Override
  public void initialize() {
    if (useAllianceColor && trajectory.fromGUI) {
      transformedTrajectory = transformTrajectory(trajectory, DriverStation.getAlliance());
    } else {
      transformedTrajectory = trajectory;
    }

    if (logActiveTrajectory != null) {
      logActiveTrajectory.accept(transformedTrajectory);
    }

    this.timer.reset();
    this.timer.start();

    PathPlannerServer.sendActivePath(transformedTrajectory.getStates());
    poseReset.accept(transformedTrajectory.getInitialPose());
  }

  @Override
  public void execute() {
    double currentTime = this.timer.get();
    Pose2d currentPose = this.poseSupplier.get();

    PathPlannerTrajectory.PathPlannerState desiredState =
        (PathPlannerTrajectory.PathPlannerState) transformedTrajectory.sample(currentTime);
    PathPlannerTrajectory.PathPlannerState nextDesiredState =
        (PathPlannerTrajectory.PathPlannerState)
            transformedTrajectory.sample(currentTime + 0.02);

    var currentWheelSpeeds =
        kinematics.toWheelSpeeds(
            new ChassisSpeeds(
                desiredState.velocityMetersPerSecond, 0, desiredState.angularVelocityRadPerSec));
    var nextWheelSpeeds =
        kinematics.toWheelSpeeds(
            new ChassisSpeeds(
                nextDesiredState.velocityMetersPerSecond,
                0,
                nextDesiredState.angularVelocityRadPerSec));

    PathPlannerServer.sendPathFollowingData(desiredState.poseMeters, currentPose);


    DifferentialDriveWheelSpeeds speeds = speedsSupplier.get();

    DifferentialDriveWheelVoltages feedbackVoltages =
        controller.calculate(
            currentPose,
            speeds.leftMetersPerSecond,
            speeds.rightMetersPerSecond,
            desiredState);

    DifferentialDriveWheelVoltages feedforwardVoltages =
        feedforward.calculate(
            currentWheelSpeeds.leftMetersPerSecond,
            nextWheelSpeeds.leftMetersPerSecond, // THIS WAS BACKWARDS
            currentWheelSpeeds.rightMetersPerSecond,
            nextWheelSpeeds.rightMetersPerSecond,
            0.02);

    DifferentialDriveWheelVoltages targetDifferentialDriveWheelVoltages;
    if (disableController) {
      targetDifferentialDriveWheelVoltages =
          new DifferentialDriveWheelVoltages(
              feedforwardVoltages.left,
              feedforwardVoltages.right);
    } else {
      targetDifferentialDriveWheelVoltages =
          new DifferentialDriveWheelVoltages(
              feedforwardVoltages.left + feedbackVoltages.left,
              feedforwardVoltages.right + feedbackVoltages.right);
    }

    this.output.accept(targetDifferentialDriveWheelVoltages);

    if (logTargetPose != null) {
      logTargetPose.accept(desiredState.poseMeters);
    }

    if (logError != null) {
      logError.accept(
          currentPose.getTranslation().minus(desiredState.poseMeters.getTranslation()),
          currentPose.getRotation().minus(desiredState.poseMeters.getRotation()));
    }

    if (logSetpoint != null) {
      logSetpoint.accept(targetDifferentialDriveWheelVoltages);
    }

    if (logMeasuredSpeeds != null) {
      logMeasuredSpeeds.accept(speeds);
    }

    if (logTargetSpeeds != null) {
      logTargetSpeeds.accept(new ChassisSpeeds(
          desiredState.velocityMetersPerSecond, 
          0, 
          desiredState.angularVelocityRadPerSec));
    }
  }

  @Override
  public void end(boolean interrupted) {
    this.timer.stop();

    if (interrupted
        || Math.abs(transformedTrajectory.getEndState().velocityMetersPerSecond) < 0.1) {

      this.output.accept(new DifferentialDriveWheelVoltages());
    }
  }

  @Override
  public boolean isFinished() {
    return this.timer.hasElapsed(transformedTrajectory.getTotalTimeSeconds());
  }

  private static void defaultLogError(Translation2d translationError, Rotation2d rotationError) {
    SmartDashboard.putNumber("PPLTVDiffControllerCommand/xErrorMeters", translationError.getX());
    SmartDashboard.putNumber("PPLTVDiffControllerCommand/yErrorMeters", translationError.getY());
    SmartDashboard.putNumber(
        "PPLTVDiffControllerCommand/rotationErrorDegrees", rotationError.getDegrees());
  }

  /**
   * Set custom logging callbacks for this command to use instead of the default configuration of
   * pushing values to SmartDashboard
   *
   * @param logActiveTrajectory Consumer that accepts a PathPlannerTrajectory representing the
   *     active path. This will be called whenever a PPRamseteCommand starts
   * @param logTargetPose Consumer that accepts a Pose2d representing the target pose while path
   *     following
   * @param logSetpoint Consumer that accepts a ChassisSpeeds object representing the setpoint
   *     speeds
   * @param logError BiConsumer that accepts a Translation2d and Rotation2d representing the error
   *     while path following
   */
  public static void setLoggingCallbacks(
      Consumer<PathPlannerTrajectory> logActiveTrajectory,
      Consumer<Pose2d> logTargetPose,
      Consumer<DifferentialDriveWheelVoltages> logSetpoint,
      BiConsumer<Translation2d, Rotation2d> logError,
      Consumer<DifferentialDriveWheelSpeeds> logMeasuredSpeeds,
      Consumer<ChassisSpeeds> logTargetSpeeds) {
    PPLTVDiffControllerCommand.logActiveTrajectory = logActiveTrajectory;
    PPLTVDiffControllerCommand.logTargetPose = logTargetPose;
    PPLTVDiffControllerCommand.logSetpoint = logSetpoint;
    PPLTVDiffControllerCommand.logError = logError;
    PPLTVDiffControllerCommand.logMeasuredSpeeds = logMeasuredSpeeds;
    PPLTVDiffControllerCommand.logTargetSpeeds = logTargetSpeeds;
  }

  public static PathPlannerTrajectory transformTrajectory(
      PathPlannerTrajectory trajectory, DriverStation.Alliance alliance) {
    if (alliance == DriverStation.Alliance.Red) {
      List<State> transformedStates = new ArrayList<>();

      for (State s : trajectory.getStates()) {
        PathPlannerState state = (PathPlannerState) s;

        transformedStates.add(transformState(state, alliance));
      }

      return new PathPlannerTrajectory(
          transformedStates,
          trajectory.getMarkers(),
          trajectory.getStartStopEvent(),
          trajectory.getEndStopEvent(),
          trajectory.fromGUI);
    } else {
      return trajectory;
    }
  }

  private static PathPlannerState transformState(
      PathPlannerState state, DriverStation.Alliance alliance) {
    if (alliance == DriverStation.Alliance.Red) {
      // Create a new state so that we don't overwrite the original
      PathPlannerState transformedState = new PathPlannerState();

      Translation2d transformedTranslation =
          new Translation2d(16.54 - state.poseMeters.getX(), state.poseMeters.getY());
      Rotation2d transformedHeading = new Rotation2d(Math.PI).minus(state.poseMeters.getRotation());
      Rotation2d transformedHolonomicRotation =
          new Rotation2d(Math.PI).minus(state.holonomicRotation);

      transformedState.timeSeconds = state.timeSeconds;
      transformedState.velocityMetersPerSecond = state.velocityMetersPerSecond;
      transformedState.accelerationMetersPerSecondSq = state.accelerationMetersPerSecondSq;
      transformedState.poseMeters = new Pose2d(transformedTranslation, transformedHeading);
      transformedState.angularVelocityRadPerSec = -state.angularVelocityRadPerSec;
      transformedState.holonomicRotation = transformedHolonomicRotation;
      transformedState.holonomicAngularVelocityRadPerSec = -state.holonomicAngularVelocityRadPerSec;
      transformedState.curvatureRadPerMeter = -state.curvatureRadPerMeter;

      return transformedState;
    } else {
      return state;
    }
  }

  public static void setControllerDisabled(boolean disable) {
    disableController = disable;
  }
}
