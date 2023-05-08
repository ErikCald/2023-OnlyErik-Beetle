package frc.robot.auto;

import java.io.IOException;
import java.util.ArrayList;
import java.util.function.BiConsumer;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class AprilTagVision {
    
    private Field2d m_field2d;

    private ArrayList<PhotonCameraWrapper> cameras;
    private AprilTagFieldLayout fieldLayout;

    public AprilTagVision(BiConsumer<Pose2d, Double> addVisionMeasurement) {
        m_field2d = new Field2d();

        try {
            // Attempt to load the AprilTagFieldLayout that will tell us where the tags are on the field.
            fieldLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
        } catch (IOException e) {
            // The AprilTagFieldLayout failed to load. We won't be able to estimate poses if we don't know
            // where the tags are.
            DriverStation.reportError("Failed to load AprilTagFieldLayout. AprilTag vision disabled.", e.getStackTrace());
        }

        if (fieldLayout != null) {
            cameras.add(new PhotonCameraWrapper(
                "camera1", // PhotoCamera name
                "camera1TagPoses", // Name for logging poses of all tags the camera sees
                new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0)), // Location of camera on robot
                m_field2d,
                addVisionMeasurement, 
                fieldLayout
            ));
        }
    }

    public void update(Pose2d prevPoseEstimate) {
        m_field2d.setRobotPose(prevPoseEstimate);

        if (fieldLayout == null) {
            return;
        }

        for (PhotonCameraWrapper camera : cameras) {
            camera.update(prevPoseEstimate);
        }
    }
}
