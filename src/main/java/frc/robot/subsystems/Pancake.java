// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.PancakeConstants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pancake extends SubsystemBase implements Loggable {
  private CANSparkMax rotationMotor;
  private RelativeEncoder rotationEncoder;

  /** Creates a new Pancake. */
  public Pancake() {
    rotationMotor = new CANSparkMax(PancakeConstants.rotationPort, MotorType.kBrushless);

    rotationEncoder = rotationMotor.getEncoder();
  }

  public void setRotationSpeed(double speed) {
    rotationMotor.set(speed);
  }

  public void stopRotation() {
    rotationMotor.set(0);
  }

  @Log
  public double getRotation() {
    return rotationEncoder.getPosition();
  }

  public void resetRotationHeading() {
    rotationEncoder.setPosition(0);
  }
}
