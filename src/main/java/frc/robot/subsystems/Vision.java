// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  NetworkTable table;
  NetworkTableEntry tx;
  NetworkTableEntry ty;
  NetworkTableEntry ta;
  NetworkTableEntry fid;

  /** Creates a new Vision. */
  public Vision() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
    fid = table.getEntry("tv");
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("LimelightX", tx.getDouble(0.0));
    SmartDashboard.putNumber("LimelightY", ty.getDouble(0.0));
    SmartDashboard.putNumber("LimelightArea", ta.getDouble(0.0));
    SmartDashboard.putNumber("LimelightFid", fid.getDouble(0.0));
  }

  public double getTagID() {
    return fid.getDouble(0.0);
  }

  public double getTargetX() {
    return tx.getDouble(0.0);
  }

  public double getTargetY() {
    return ty.getDouble(0.0);
  }

  public double getTargetArea() {
    return ta.getDouble(0.0);
  }
}
