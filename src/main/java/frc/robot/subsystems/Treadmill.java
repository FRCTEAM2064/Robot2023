// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TreadmillConstants;;

public class Treadmill extends SubsystemBase {
  private CANSparkMax motor;

  /** Creates a new Treadmill. */
  public Treadmill() {
    motor = new CANSparkMax(TreadmillConstants.treadmillId, MotorType.kBrushless);
    SmartDashboard.putData("Buttons/Start Treadmill", new InstantCommand(() -> startTreadmill()));
    SmartDashboard.putData("Buttons/Stop Treadmill", new InstantCommand(() -> stopTreadmill()));
    SmartDashboard.putData("Buttons/Reverse Treadmill", new InstantCommand(() -> reverseTreadmill()));
  }

  public void startTreadmill() {
    motor.set(TreadmillConstants.treadmillSpeed);
  }

  public void stopTreadmill() {
    motor.set(0);
  }

  public void reverseTreadmill() {
    motor.set(-1 * TreadmillConstants.treadmillSpeed);
  }
}
