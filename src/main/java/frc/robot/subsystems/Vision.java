// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {

  private PhotonCamera camera;

  /** Creates a new Vision. */
  public Vision() {
    try {
      camera = new PhotonCamera("Global_Shutter_Camera");
    } catch (Exception e) {
      System.out.println("L");
    }
  }

  @Override
  public void periodic() {
    if (camera == null) {
      return;
    }
    if (camera.getLatestResult() != null && camera.getLatestResult().getBestTarget() != null) {
      SmartDashboard.putNumber("Best Target Yaw", camera.getLatestResult().getBestTarget().getYaw());
    }
  }

  public PhotonPipelineResult getLatestResult() {
    return camera.getLatestResult();
  }

  public PhotonTrackedTarget getBestTarget() {
    return getLatestResult().getBestTarget();
  }
}
