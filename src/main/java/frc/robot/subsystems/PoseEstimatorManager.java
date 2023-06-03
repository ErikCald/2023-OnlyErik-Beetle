// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.lib2706.AdvantageUtil;
import frc.lib.lib2706.DifferentialDrivePoseEstimatorExposed;
import frc.robot.Config.VISION;
import frc.robot.Config.VISION.APRILTAG;

/** Add your docs here. */
public class PoseEstimatorManager {
    private class PoseEstimatorData {
        private DifferentialDrivePoseEstimatorExposed estimator;
        private String name;
        private BooleanSubscriber enabled;
        private BooleanSubscriber disableVisionFeedback;
        private DoubleArrayPublisher pubEstPose;
        private DoubleArrayPublisher pubVisionPose;

        private PoseEstimatorData(DifferentialDrivePoseEstimatorExposed estimator, String name) {
            this.estimator = estimator;
            this.name = name;

            NetworkTable table = NetworkTableInstance.getDefault().getTable("PoseEstimators/"+name);
            enabled = table.getBooleanTopic("enabled").subscribe(true);
            disableVisionFeedback = table.getBooleanTopic("disableVisionFeedback").subscribe(true);
            pubEstPose = table.getDoubleArrayTopic("estPose").publish(PubSubOption.periodic(0.02));
            pubVisionPose = table.getDoubleArrayTopic("visionPose").publish(PubSubOption.periodic(0.02));
        }
    }

    private PoseEstimatorData primaryEstimator;
    private ArrayList<PoseEstimatorData> otherEstimators;

    public PoseEstimatorManager(
            DifferentialDriveKinematics kinematics,
            Rotation2d gyroAngle,
            double leftDistanceMeters,
            double rightDistanceMeters,
            Pose2d initialPoseMeters) {

        primaryEstimator = new PoseEstimatorData(
            new DifferentialDrivePoseEstimatorExposed(
                kinematics, 
                gyroAngle, 
                leftDistanceMeters, 
                rightDistanceMeters, 
                initialPoseMeters), 
            "primary");

        otherEstimators = new ArrayList<>();

        otherEstimators.add(new PoseEstimatorData(
            new DifferentialDrivePoseEstimatorExposed(
                kinematics, 
                gyroAngle, 
                leftDistanceMeters, 
                rightDistanceMeters, 
                initialPoseMeters), 
            APRILTAG.PHOTON_CAMERA_NAMES[0]));

        otherEstimators.add(new PoseEstimatorData(
            new DifferentialDrivePoseEstimatorExposed(
                kinematics, 
                gyroAngle, 
                leftDistanceMeters, 
                rightDistanceMeters, 
                initialPoseMeters), 
            APRILTAG.PHOTON_CAMERA_NAMES[1]));

        otherEstimators.add(new PoseEstimatorData(
            new DifferentialDrivePoseEstimatorExposed(
                kinematics, 
                gyroAngle, 
                leftDistanceMeters, 
                rightDistanceMeters, 
                initialPoseMeters), 
            APRILTAG.PHOTON_CAMERA_NAMES[2]));

    }

    public Pose2d update(Rotation2d gyroAngle, double distanceLeftMeters, double distanceRightMeters) {
        for (PoseEstimatorData data : otherEstimators) {
            if (data.enabled.getAsBoolean()) {
                data.pubEstPose.accept(AdvantageUtil.deconstruct(
                    data.estimator.update(gyroAngle, distanceLeftMeters, distanceRightMeters)));
            }
        }

        Pose2d primaryPose = primaryEstimator.estimator.update(gyroAngle, distanceLeftMeters, distanceRightMeters);
        primaryEstimator.pubEstPose.accept(AdvantageUtil.deconstruct(primaryPose));
        return primaryPose;
    }

    private void setPrimaryVisionPose(Pose2d pose, double timestamp) {
        primaryEstimator.pubVisionPose.accept(AdvantageUtil.deconstruct(pose));
        if (primaryEstimator.disableVisionFeedback.getAsBoolean() == false) {
            primaryEstimator.estimator.addVisionMeasurement(pose, timestamp);
        }
    }

    public void setVisionPose(Pose2d pose, double timestamp, String name) {
        setPrimaryVisionPose(pose, timestamp);

        for (PoseEstimatorData data : otherEstimators) {
            if (data.name == name) {
                data.pubVisionPose.accept(AdvantageUtil.deconstruct(pose));
                if (data.disableVisionFeedback.getAsBoolean() == false) {
                    data.estimator.addVisionMeasurement(pose, timestamp);
                }
                return;
            }
        }
        DriverStation.reportError("PoseEstimatorManager: setOtherVisionPose was called and given a name that doesn't exist", null);
    }

    public Optional<Pose2d> getPoseAtTimestamp(double timestamp, String name) {
        for (PoseEstimatorData data : otherEstimators) {
            if (data.name == name) {
                return data.estimator.getPoseAtTimestamp(timestamp);
            }
        }
        DriverStation.reportError("PoseEstimatorManager: getPoseAtTimestamp was called and given a name that doesn't exist", null);
        
        return Optional.empty();
    }

    public Pose2d getPrimaryPose() {
        return primaryEstimator.estimator.getEstimatedPosition();
    }

    public void resetPose(
            Rotation2d gyroAngle,
            double leftPositionMeters,
            double rightPositionMeters,
            Pose2d poseMeters) {
        for (PoseEstimatorData data : otherEstimators) {
            data.estimator.resetPosition(gyroAngle, leftPositionMeters, rightPositionMeters, poseMeters);
        }
    }
}
