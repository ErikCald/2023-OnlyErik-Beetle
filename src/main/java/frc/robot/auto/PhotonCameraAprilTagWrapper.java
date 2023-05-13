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

import java.sql.Driver;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Optional;
import java.util.function.BiConsumer;
import java.util.function.Function;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSub;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.lib686.AdvantageUtil;
import frc.robot.Config.GENERAL;
import frc.robot.Config.VISION.APRILTAG;

public class PhotonCameraAprilTagWrapper {
    private AdvScopeAprilTagPhotonCamera m_advScopeDisplay;

    private String m_cameraName;
    private Transform3d m_robotToCamera;
    private Function<Double, Optional<Pose2d>> m_getPoseAtTimestamp;
    private BiConsumer<Pose2d, Double> m_addVisionMeasurement;
    private AprilTagFieldLayout m_fieldLayout;
    private PhotonCamera m_photonCamera;
    private PhotonPoseEstimator m_photonPoseEstimator;

    private DoubleArrayPublisher pubEstPose;

    private double prevTimestamp = 0;
    private double prevPipelineTimestamp = -1;
    

    public PhotonCameraAprilTagWrapper(String cameraName, Transform3d robotToCamera, Function<Double, Optional<Pose2d>> getPoseAtTimestamp, BiConsumer<Pose2d, Double> addVisionMeasurement, AprilTagFieldLayout fieldLayout) {
        m_advScopeDisplay = new AdvScopeAprilTagPhotonCamera(cameraName, robotToCamera, fieldLayout);
        
        m_cameraName = cameraName;
        m_robotToCamera = robotToCamera;
        m_addVisionMeasurement = addVisionMeasurement;
        m_getPoseAtTimestamp = getPoseAtTimestamp;
        m_fieldLayout = fieldLayout;

        // Change the name of your camera here to whatever it is in the PhotonVision UI.
        m_photonCamera = new PhotonCamera(cameraName);
            
        // Create pose estimator
        m_photonPoseEstimator =
                new PhotonPoseEstimator(
                        fieldLayout, PoseStrategy.MULTI_TAG_PNP, m_photonCamera, robotToCamera);
        m_photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        pubEstPose = NetworkTableInstance.getDefault().getTable("AprilTags").getDoubleArrayTopic("EstPoseV2").publish(PubSubOption.periodic(0.02));
    }

    public void update(Pose2d prevEstimatedRobotPose) {
        m_photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);

        Optional<EstimatedRobotPose> result;
        if (APRILTAG.DISABLE_TAG_TRIMMING) {
            result = m_photonPoseEstimator.update();
        } else {
            PhotonPipelineResult pipelineResult = m_photonCamera.getLatestResult();
            
            // Check if the timestamp is invalid or stale.
            double pipelineTimestamp = pipelineResult.getTimestampSeconds();
            if (pipelineTimestamp == -1 || areFloatsEqual(pipelineTimestamp, prevPipelineTimestamp)) {
                m_advScopeDisplay.noNewData();
                return;
            } else {
                prevPipelineTimestamp = pipelineTimestamp;
            }

            // Filter the pipeline
            // testFilterPipelineResult(pipelineResult);
            filterPipelineResult(pipelineResult);

            result = m_photonPoseEstimator.update(pipelineResult);
        }
        
        /*
         * No targets in view
         */
        if (result.isEmpty()) {
            m_advScopeDisplay.noNewData();
            return;
        }
        
        EstimatedRobotPose estimatedRobotPose = result.get();
        pubEstPose.accept(AdvantageUtil.deconstruct(estimatedRobotPose.estimatedPose.toPose2d()));

        Optional<Pose2d> optionalPoseAtTimestamp = m_getPoseAtTimestamp.apply(estimatedRobotPose.timestampSeconds);
        if (optionalPoseAtTimestamp.isEmpty()) {
            DriverStation.reportError("AprilTags on " + m_cameraName + " failed because poseAtTimestamp is empty.", false);
            m_advScopeDisplay.noNewData();
            return;
        }

        Pose2d poseAtTimestamp = optionalPoseAtTimestamp.get();
        

        // THIS IS ALREADY HANDLED BY PhotonPoseEstimator
        // /*
        //  * Check if the data is new data or stale data.
        //  */
        // if (areFloatsEqual(estimatedRobotPose.timestampSeconds, prevTimestamp)) {
        //     // The timestamp hasn't change since the last roborio cycle
        //     m_advScopeDisplay.noNewData();
        //     return;
        // } else {
        //     // The data is new
        //     prevTimestamp = estimatedRobotPose.timestampSeconds;
        // }

        /*
         * Check if the data is valid (check for discrepancies in the data).
         */
        if (checkValid(estimatedRobotPose, prevEstimatedRobotPose, poseAtTimestamp)) {

            // Data is accepted, pass estimated pose to a WPILib PoseEstimator
            m_addVisionMeasurement.accept(estimatedRobotPose.estimatedPose.toPose2d(), 
                    estimatedRobotPose.timestampSeconds);

            // Update AdvantageScope display
            m_advScopeDisplay.dataAccepted(estimatedRobotPose, poseAtTimestamp);
        } else {
            // Data is discarded, update AdvantageScope display
            m_advScopeDisplay.dataDiscarded(estimatedRobotPose, poseAtTimestamp);
        }
    }

    private void testFilterPipelineResult(PhotonPipelineResult pipelineResult) {
        double pipelineTimestamp = pipelineResult.getTimestampSeconds();

        if (pipelineTimestamp == -1) {
            // Skip tag trimming
            return;// pipelineResult;
        } 

        Optional<Pose2d> optionalPoseAtTimestamp = m_getPoseAtTimestamp.apply(pipelineTimestamp);

        if (optionalPoseAtTimestamp.isEmpty()) {
            // Skip tag trimming
            return;// pipelineResult;
        }
        final int tagToRemove = 2;
        List<Pose3d> tagRemovedPose = new ArrayList<>();
        List<Integer> tagRemovedID = new ArrayList<>();
        Iterator<PhotonTrackedTarget> iterator = pipelineResult.targets.iterator();
        while (iterator.hasNext()) {
            PhotonTrackedTarget target = iterator.next();
            
            if (target.getFiducialId() == tagToRemove) {
                Pose3d bestTag = new Pose3d(optionalPoseAtTimestamp.get()).transformBy(m_robotToCamera).transformBy(target.getBestCameraToTarget());
                tagRemovedPose.add(bestTag);
                tagRemovedID.add(tagToRemove);
                iterator.remove();
                break;
            }
        }

        m_advScopeDisplay.logRemovedTags(tagRemovedPose, tagRemovedID);
    }

    private void filterPipelineResult(PhotonPipelineResult pipelineResult) {
        if (APRILTAG.DISABLE_TAG_TRIMMING) {
            // Skip tag trimming
            return;
        }

        double pipelineTimestamp = pipelineResult.getTimestampSeconds();

        if (pipelineTimestamp == -1) { // || areFloatsEqual(pipelineTimestamp, prevPipelineTimestamp)) {
            // Skip tag trimming
            return;
        } 

        Optional<Pose2d> optionalPoseAtTimestamp = m_getPoseAtTimestamp.apply(pipelineTimestamp);

        if (optionalPoseAtTimestamp.isEmpty()) {
            // Skip tag trimming
            return;
        }

        Pose2d poseAtTimeStamp = optionalPoseAtTimestamp.get();
        Pose3d fieldToCam = new Pose3d(poseAtTimeStamp).transformBy(m_robotToCamera);

        Iterator<PhotonTrackedTarget> iterator = pipelineResult.targets.iterator();

        List<Pose3d> removedTagsPoses = new ArrayList<>();
        List<Integer> removedTagsIDs = new ArrayList<>();
        while (iterator.hasNext()) {
            PhotonTrackedTarget target = iterator.next();
            
            Optional<Pose3d> tag = m_fieldLayout.getTagPose(target.getFiducialId());

            Pose3d bestTag = fieldToCam.transformBy(target.getBestCameraToTarget());

            if (tag.isEmpty()) {
                removedTagsPoses.add(bestTag);
                removedTagsIDs.add(target.getFiducialId());
                iterator.remove();
                continue;
            }

            if (comparePoses(bestTag, tag.get(), APRILTAG.ALLOWABLE_TAG_DISTANCE_ERROR, APRILTAG.ALLOWABLE_TAG_ANGLE_ERROR)) {
                continue; // BestTag is good, continue to the next tag
            }

            Pose3d altTag = fieldToCam.transformBy(target.getAlternateCameraToTarget());

            if (comparePoses(altTag, tag.get(), APRILTAG.ALLOWABLE_TAG_DISTANCE_ERROR, APRILTAG.ALLOWABLE_TAG_ANGLE_ERROR)) {
                continue; // AltTag is good, continue to the next tag
            }

            removedTagsPoses.add(bestTag);
            removedTagsIDs.add(target.getFiducialId());
            iterator.remove();            
        }

        m_advScopeDisplay.logRemovedTags(removedTagsPoses, removedTagsIDs);
    }

    private boolean checkValid(EstimatedRobotPose estimatedRobotPose, Pose2d prevEstimatedPose, Pose2d poseAtTimestamp) {
        // If vision feedback is disabled then let everything through for the display
        if (APRILTAG.DISABLE_VISION_FEEDBACK) {
            return true;
        }

        // Don't accept the data if it's more than 1 meter off of the previously estimated pose
        if (Math.abs(estimatedRobotPose.estimatedPose.toPose2d().getTranslation().getDistance(poseAtTimestamp.getTranslation())) 
                > APRILTAG.ALLOWABLE_POSE_DISTANCE_ERROR) {
            return false;
        }
        
        // All checks passed
        return true;
    }

    private boolean areFloatsEqual(double val1, double val2) {
        return areFloatsEqual(val1, val2, GENERAL.FLOAT_EQUALS_TOLARENCE);
    }

    private boolean areFloatsEqual(double val1, double val2, double tolarence) {
        return Math.abs(val1 - val2) < tolarence;
    }

    private boolean comparePoses(Pose3d pose1, Pose3d pose2, double distanceTolarence, double angleTolarence) {
        return pose1.getTranslation().getDistance(pose2.getTranslation()) < distanceTolarence &&
                compareQuaternion(pose1.getRotation().getQuaternion(), pose2.getRotation().getQuaternion(), angleTolarence);
    }

    private static boolean compareQuaternion(Quaternion q1, Quaternion q2, double tolerence) {
        double r = q1.getW();
        Vector<N3> v = VecBuilder.fill(q1.getX(), q1.getY(), q1.getZ());

        double other_r = q2.getW();
        Vector<N3> other_v = VecBuilder.fill(q2.getX(), q2.getY(), q2.getZ());

        return Math.abs(r * other_r + v.dot(other_v)) > 1.0 - tolerence;
    }


    // public static void main(String[] args) {
    //     Rotation3d rot1 = new Rotation3d(Units.degreesToRadians(100), Units.degreesToRadians(0), Units.degreesToRadians(0));
    //     Rotation3d rot2;
        
    //     int i = 0;
    //     do {
    //         rot2 = new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(i), Units.degreesToRadians(i));
    //         System.out.println(i);
    //         i++;


    //     } while (compareQuaternion(rot1.getQuaternion(), rot2.getQuaternion(), 0.003));//APRILTAG.ALLOWABLE_TAG_ANGLE_ERROR));
        

    //     System.out.println(i);
    // }
}
