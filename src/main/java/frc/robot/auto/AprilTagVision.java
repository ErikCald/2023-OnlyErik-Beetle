package frc.robot.auto;

import java.util.ArrayList;
import java.util.Optional;
import java.util.function.BiConsumer;
import java.util.function.Function;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Config.VISION.APRILTAG;

public class AprilTagVision {
    
    private Field2d m_field2d;

    private ArrayList<PhotonCameraAprilTagWrapper> cameras;
    private AprilTagFieldLayout fieldLayout;

    public AprilTagVision(Function<Double, Optional<Pose2d>> getPoseAtTimestamp, BiConsumer<Pose2d, Double> addVisionMeasurement) {
        cameras = new ArrayList<>();
        m_field2d = new Field2d();

        SmartDashboard.putData("field2d", m_field2d);
        fieldLayout = APRILTAG.FIELDLAYOUT_ERIK_BOTTOM_FLOOR; 
        // try {
        //     // Attempt to load the AprilTagFieldLayout that will tell us where the tags are on the field.
        //     // fieldLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();

        //     // fieldLayout = new AprilTagFieldLayout("frc/robot/auto/erikTopFloorTest1.json");//"apriltagfieldlayouts/erikTopFloorTest1.json");//new AprilTagFieldLayout("apriltagfieldlayouts/erikTopFloorTest1.json");
        //     // fieldLayout = new AprilTagFieldLayout(Filesystem.getDeployDirectory().getPath() + "erikTopFloorTest1.json");
            
        //     System.out.println(fieldLayout);
        // } catch (IOException e) {
        //     // The AprilTagFieldLayout failed to load. We won't be able to estimate poses if we don't know
        //     // where the tags are.
        //     DriverStation.reportError("Failed to load AprilTagFieldLayout. AprilTag vision disabled.", e.getStackTrace());
        //     fieldLayout = null;
        // }

        if (fieldLayout != null) {
            if (APRILTAG.PHOTON_CAMERA_NAMES.length != APRILTAG.PHOTON_CAMERA_LOCATIONS.length) {
                DriverStation.reportError("Number of photon camera names does not match the number of camera locations. " + 
                        "Photon AprilTag vision disabled.", true);
                fieldLayout = null;
                return;
            }

            for (int i = 0; i < APRILTAG.PHOTON_CAMERA_NAMES.length; i++) {
                cameras.add(new PhotonCameraAprilTagWrapper(
                    APRILTAG.PHOTON_CAMERA_NAMES[i], // PhotoCamera name
                    APRILTAG.PHOTON_CAMERA_LOCATIONS[i], // Location of camera on robot
                    getPoseAtTimestamp,
                    addVisionMeasurement, 
                    fieldLayout
                ));
            }
        }
    }

    public void update(Pose2d prevPoseEstimate) {
        m_field2d.setRobotPose(prevPoseEstimate);

        if (fieldLayout == null) {
            return;
        }

        for (PhotonCameraAprilTagWrapper camera : cameras) {
            camera.update(prevPoseEstimate);
        }
    }

    public void setActiveTrajectory(PathPlannerTrajectory traj) {
        m_field2d.getObject("traj").setTrajectory((Trajectory) traj);
    }
}
