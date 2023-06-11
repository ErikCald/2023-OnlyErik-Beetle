// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.libLimelight.LimelightHelpers;

public class TakeSnapshots extends CommandBase {

    PhotonCamera[] cameras;
    int count = 0;
    /** Creates a new TakeSnapshots. */
    public TakeSnapshots() {
        // Use addRequirements() here to declare subsystem dependencies.

        cameras = new PhotonCamera[]{
            new PhotonCamera("Microsoft_LifeCam_HD-3000"),
            new PhotonCamera("HD_USB_Camera"),
            new PhotonCamera("OV9281")
        };
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        for (int i = 0; i < cameras.length; i++) {
            cameras[i].takeOutputSnapshot();
        }

        LimelightHelpers.takeSnapshot("limelight", "snapshot" + count++);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
