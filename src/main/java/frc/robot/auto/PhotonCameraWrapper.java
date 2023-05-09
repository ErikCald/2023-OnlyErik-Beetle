/*
 * MIT License
 *
 * Copyright (c) PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package frc.robot.auto;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.BiConsumer;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.util.datalog.IntegerArrayLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.lib.lib686.AdvantageUtil;
import frc.robot.Config.GENERAL;
import frc.robot.Config.VISION.APRILTAG;

public class PhotonCameraWrapper {
    private Field2d m_field2d;
    private String estimatedPoseNTName;
    private DoubleArrayLogEntry m_tagPosesLog;
    private IntegerArrayLogEntry m_tagIDsLog;
    private Transform3d m_robotToCamera;
    private BiConsumer<Pose2d, Double> m_addVisionMeasurement;
    private PhotonCamera photonCamera;
    private PhotonPoseEstimator photonPoseEstimator;

    private double prevTimestamp = 0;

    public PhotonCameraWrapper(String cameraName, Transform3d robotToCamera, Field2d field2d, BiConsumer<Pose2d, Double> addVisionMeasurement, AprilTagFieldLayout fieldLayout) {
        m_field2d = field2d;
        m_addVisionMeasurement = addVisionMeasurement;
        estimatedPoseNTName = cameraName + " EstPose";
        m_robotToCamera = robotToCamera;

        DataLog log = DataLogManager.getLog();
        m_tagPosesLog = new DoubleArrayLogEntry(log, cameraName + "TagPoses");
        m_tagIDsLog = new IntegerArrayLogEntry(log, cameraName + "TagIDs");

        // Change the name of your camera here to whatever it is in the PhotonVision UI.
        photonCamera = new PhotonCamera(cameraName);
            
        // Create pose estimator
        photonPoseEstimator =
                new PhotonPoseEstimator(
                        fieldLayout, PoseStrategy.MULTI_TAG_PNP, photonCamera, robotToCamera);
        photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    }

    /**
     * @param estimatedRobotPose The current best guess at robot pose
     * @return an EstimatedRobotPose with an estimated pose, the timestamp, and targets used to create
     *     the estimate
     */
    public void update(Pose2d prevEstimatedRobotPose) {
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);

        Optional<EstimatedRobotPose> result = photonPoseEstimator.update();

        if (result.isPresent() && checkValid(result, prevEstimatedRobotPose)) {
            EstimatedRobotPose estimatedRobotPose = result.get();
            m_addVisionMeasurement.accept(estimatedRobotPose.estimatedPose.toPose2d(), 
                    estimatedRobotPose.timestampSeconds);
        
            m_field2d.getObject(estimatedPoseNTName).setPose(estimatedRobotPose.estimatedPose.toPose2d());

            List<PhotonTrackedTarget> targets = estimatedRobotPose.targetsUsed;
            List<Pose3d> targetPoses = new ArrayList<>();
            List<Integer> targetTagIDs = new ArrayList<>();
            for (PhotonTrackedTarget target : targets) {
                targetPoses.add(new Pose3d(prevEstimatedRobotPose).transformBy(m_robotToCamera).transformBy(target.getBestCameraToTarget()));
                targetTagIDs.add(target.getFiducialId());
            }
            m_tagPosesLog.append(AdvantageUtil.deconstructPose3ds(targetPoses));
            m_tagIDsLog.append(convertIntegers(targetTagIDs));
                    
        } else {
            // move it way off the screen to make it disappear
            m_field2d.getObject(estimatedPoseNTName).setPose(new Pose2d(-100, -100, new Rotation2d()));
            m_tagPosesLog.append(new double[]{});
            m_tagIDsLog.append(new long[]{});
        }
    }

    private boolean checkValid(Optional<EstimatedRobotPose> result, Pose2d prevEstimatedPose) {
        if (result.isEmpty()) {
            return false;
        }

        EstimatedRobotPose estimatedRobotPose = result.get();

        if (areFloatsEqual(estimatedRobotPose.timestampSeconds, prevTimestamp)) {
            prevTimestamp = estimatedRobotPose.timestampSeconds;
        } else {
            return false;
        }

        if (Math.abs(estimatedRobotPose.estimatedPose.toPose2d().getTranslation().getDistance(prevEstimatedPose.getTranslation())) 
                     > APRILTAG.ALLOWABLE_DISTANCE_ERROR) {
            return false;
        }

        // All checks passed
        return true;
    }

    private boolean areFloatsEqual(double val1, double val2) {
        return Math.abs(val1 - val2) < GENERAL.FLOAT_EQUALS_TOLARENCE;
    }

    private long[] convertIntegers(List<Integer> integers) {
        long[] arr = new long[integers.size()];
        for (int i = 0; i < integers.size(); i++) {
            Integer value = integers.get(i);
            if (value == null) {
                DriverStation.reportError("Failed to log tag id, null integer.", true);
                arr[i] = -1;
            } else {
                arr[i] = value.longValue();
            }
        }
        return arr;
    }
}
