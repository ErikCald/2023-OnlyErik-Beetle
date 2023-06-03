package frc.robot.auto;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.IntegerArrayPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.util.datalog.IntegerArrayLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.lib2706.AdvantageUtil;

public class AdvScopeAprilTagPhotonCamera {
    private static final double TAG_LINGERING_DURATION = 2;
    private Timer m_timer;
    private boolean m_noData = false;
    private Transform3d m_robotToCamera;
    private AprilTagFieldLayout m_fieldLayout;

    private DoubleArrayPubLog pubLogEstPose, pubLogAcceptedTagPoses, pubLogAllTagPoses, pubLogRemovedTagsPoses, pubLogFieldTagPose;
    private IntegerArrayPubLog pubLogAcceptedTagIDs, pubLogAllTagIDs, pubLogRemovedTagsIDs;

    public AdvScopeAprilTagPhotonCamera(String cameraName, Transform3d robotToCamera, AprilTagFieldLayout fieldLayout) {
        m_robotToCamera = robotToCamera;
        m_fieldLayout = fieldLayout;
        m_timer = new Timer();

        pubLogEstPose = new DoubleArrayPubLog(cameraName, "EstimatedPose");
        pubLogAcceptedTagPoses = new DoubleArrayPubLog(cameraName, "AcceptedTagPoses");
        pubLogAcceptedTagIDs = new IntegerArrayPubLog(cameraName, "AcceptedTagIDs");
        pubLogAllTagPoses = new DoubleArrayPubLog(cameraName, "LingeringTagPoses");
        pubLogAllTagIDs = new IntegerArrayPubLog(cameraName, "LingeringTagIDs");
        pubLogFieldTagPose = new DoubleArrayPubLog(cameraName, "FieldTagPose");

        pubLogRemovedTagsPoses = new DoubleArrayPubLog(cameraName, "RemovedTagsPoses");
        pubLogRemovedTagsIDs = new IntegerArrayPubLog(cameraName, "RemovedTagsIDs");
    }

    public void noNewData() {
        clearAcceptedData();

        if (m_noData == false) {
            m_noData = true;
            m_timer.restart();
            
        } else if (m_timer.hasElapsed(TAG_LINGERING_DURATION)) {
            // The tags have lingered for the duration, so clear them
            pubLogAllTagPoses.noData();
            pubLogAllTagIDs.noData();
            pubLogFieldTagPose.noData();

            m_timer.stop();
        }
    }

    public void dataDiscarded(EstimatedRobotPose estimatedRobotPose, Pose2d poseAtTimestamp) {
        handleNewData(estimatedRobotPose, poseAtTimestamp, false);
    }

    public void dataAccepted(EstimatedRobotPose estimatedRobotPose, Pose2d poseAtTimestamp) {
        handleNewData(estimatedRobotPose, poseAtTimestamp, true);
    }

    public void logRemovedTags(List<Pose3d> removedTagsPoses, List<Integer> removedTagsIDs) {
        if (removedTagsPoses.isEmpty() || removedTagsIDs.isEmpty()) {
            pubLogRemovedTagsPoses.noData();
            pubLogRemovedTagsIDs.noData();;
        }
        pubLogRemovedTagsPoses.accept(AdvantageUtil.deconstructPose3ds(removedTagsPoses));
        pubLogRemovedTagsIDs.accept(convertIntegers(removedTagsIDs));
    }

    private void clearAcceptedData() {
        pubLogAcceptedTagPoses.noData();
        pubLogAcceptedTagIDs.noData();
        pubLogEstPose.noData();
    }

    private void handleNewData(EstimatedRobotPose estimatedRobotPose, Pose2d poseAtTimestamp, boolean newDataAccepted) {
        if (m_noData) {
            m_noData = false;
            m_timer.stop();
            m_timer.reset();
        }
        
        
        List<PhotonTrackedTarget> targets = estimatedRobotPose.targetsUsed;

        // Convert PhotonTrackedTarget into Lists
        List<Pose3d> targetPoses = new ArrayList<>();
        List<Integer> targetTagIDs = new ArrayList<>();
        List<Pose3d> fieldTagPoses = new ArrayList<>();
        for (PhotonTrackedTarget target : targets) {
            targetPoses.add(new Pose3d(poseAtTimestamp).transformBy(m_robotToCamera).transformBy(target.getBestCameraToTarget()));
            targetTagIDs.add(target.getFiducialId());
            Optional<Pose3d> tagPose = m_fieldLayout.getTagPose(target.getFiducialId());
            if (tagPose.isPresent()) {
                fieldTagPoses.add(tagPose.get());
            }
        }

        // Convert Lists into arrays for AdvantageScope
        double[] advScopeAllTagPoses = AdvantageUtil.deconstructPose3ds(targetPoses);
        long[] advScopeAllTagIDs = convertIntegers(targetTagIDs);
        double[] advScopeFieldTagPoses = AdvantageUtil.deconstructPose3ds(fieldTagPoses);

        // Update AllTags aka "LingeringTagPoses" and "LingeringTagIDs"
        m_noData = false;
        pubLogAllTagPoses.accept(advScopeAllTagPoses);
        pubLogAllTagIDs.accept(advScopeAllTagIDs);
        pubLogFieldTagPose.accept(advScopeFieldTagPoses);

        // Check if the new data was accepted
        if (newDataAccepted) {
            // Update the accepted tags
            pubLogAcceptedTagPoses.accept(advScopeAllTagPoses);
            pubLogAcceptedTagIDs.accept(advScopeAllTagIDs);
            
            // Update the accepted pose estimate
            pubLogEstPose.accept(AdvantageUtil.deconstruct(estimatedRobotPose.estimatedPose.toPose2d()));
        } else {
            clearAcceptedData();
        }
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

    private class DoubleArrayPubLog {
        private DoubleArrayPublisher m_pub;
        private DoubleArrayLogEntry m_entry;
        
        private DoubleArrayPubLog(String cameraName, String entryName) {
            m_pub = NetworkTableInstance.getDefault().getTable("AdvScope/"+cameraName).getDoubleArrayTopic(entryName).publish(PubSubOption.periodic(0.02));
            m_entry = new DoubleArrayLogEntry(DataLogManager.getLog(), cameraName+"/"+entryName);
        }

        private void accept(double[] data) {
            m_pub.accept(data);
            m_entry.append(data);
        }

        private void noData() {
            m_pub.accept(new double[]{});
            m_entry.append(new double[]{});
        }
    }

    private class IntegerArrayPubLog {
        private IntegerArrayPublisher m_pub;
        private IntegerArrayLogEntry m_entry;
        
        private IntegerArrayPubLog(String cameraName, String entryName) {
            m_pub = NetworkTableInstance.getDefault().getTable("AdvScope/"+cameraName).getIntegerArrayTopic(entryName).publish(PubSubOption.periodic(0.02));
            m_entry = new IntegerArrayLogEntry(DataLogManager.getLog(), cameraName+"/"+entryName);
        }

        private void accept(long[] data) {
            m_pub.accept(data);
            m_entry.append(data);
        }

        private void noData() {
            m_pub.accept(new long[]{});
            m_entry.append(new long[]{});
        }
    }
}
