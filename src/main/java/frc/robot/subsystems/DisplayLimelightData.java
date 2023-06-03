// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DisplayLimelightData extends SubsystemBase {

    DoubleArraySubscriber subMegaTagPoseLimelight;
    DoubleArrayPublisher pubMegaTagPose3d;

    /** Creates a new DisplayLimelightData. */
    public DisplayLimelightData() {
        NetworkTable pubTable = NetworkTableInstance.getDefault().getTable("LimelightAdvScope");
        NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

        subMegaTagPoseLimelight = limelightTable.getDoubleArrayTopic("botpose").subscribe(new double[0], PubSubOption.periodic(0.01));
        pubMegaTagPose3d = pubTable.getDoubleArrayTopic("botpose").publish(PubSubOption.periodic(0.01));
    }

    @Override
    public void periodic() {
        // double[] arr = subMegaTagPoseLimelight.get();
        // String msg = "";

        // for (int i = 0; i < 7; i++) {
        //     msg += arr[i] + ", ";
        // }
        // // arr.toString()
        // System.out.println(msg);
        pubMegaTagPose3d.accept(
            limelightPoseToAdvScopePose(subMegaTagPoseLimelight.get())
        );
        
    }


    private Pose3d limelightPoseToPose3d(double[] limelightPose) {
        return new Pose3d(
            limelightPose[0], 
            limelightPose[1], 
            limelightPose[2], 
            new Rotation3d(
                Units.degreesToRadians(limelightPose[3]),
                Units.degreesToRadians(limelightPose[4]),
                Units.degreesToRadians(limelightPose[5])
        ));
    }

    private double[] limelightPoseToAdvScopePose(double[] limelightPose) {
        Rotation3d rot = new Rotation3d(
            Units.degreesToRadians(limelightPose[3]),
            Units.degreesToRadians(limelightPose[4]),
            Units.degreesToRadians(limelightPose[5])); 

        return new double[]{
            limelightPose[0], 
            limelightPose[1], 
            limelightPose[2],
            rot.getQuaternion().getW(),
            rot.getQuaternion().getX(),
            rot.getQuaternion().getY(),
            rot.getQuaternion().getZ()
        };
    }
}
