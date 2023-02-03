// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.PancakeConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pancake extends SubsystemBase {
  private CANSparkMax rotationMotor, tiltMotor;
  private RelativeEncoder rotationEncoder, tiltEncoder;

  /** Creates a new Pancake. */
  public Pancake() {
    rotationMotor = new CANSparkMax(PancakeConstants.rotationPort, MotorType.kBrushless);
    tiltMotor = new CANSparkMax(PancakeConstants.tiltPort, MotorType.kBrushless);

    rotationEncoder = rotationMotor.getEncoder();
    tiltEncoder = tiltMotor.getEncoder();

    tiltMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    tiltMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    tiltMotor.setSoftLimit(SoftLimitDirection.kForward, PancakeConstants.maxTiltValue);
    tiltMotor.setSoftLimit(SoftLimitDirection.kForward, PancakeConstants.minTiltValue);
  }

  public void setRotationSpeed(double speed) {
    rotationMotor.set(speed);
  }

  public void stopRotation() {
    rotationMotor.set(0);
  }

  public double getRotation() {
    return rotationEncoder.getPosition();
  }

  public void resetRotationHeading() {
    rotationEncoder.setPosition(0);
  }

  public void setTiltSpeed(double speed) {
    tiltMotor.set(speed);
  }

  public void stopTilt() {
    tiltMotor.set(0);
  }

  public double getTilt() {
    return tiltEncoder.getPosition();
  }

  public void resetTiltHeading() {
    tiltEncoder.setPosition(0);
  }
}
