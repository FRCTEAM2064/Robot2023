// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.ElevatorConstants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase implements Loggable {
  private CANSparkMax winchMotor;
  private RelativeEncoder winchEncoder;

  private CANSparkMax gripperMotor;
  private RelativeEncoder gripperEncoder;

  /** Creates a new Elevator. */
  public Elevator() {
    winchMotor = new CANSparkMax(ElevatorConstants.winchPort, MotorType.kBrushless);
    winchEncoder = winchMotor.getEncoder();

    winchMotor.enableSoftLimit(SoftLimitDirection.kForward, false);
    winchMotor.enableSoftLimit(SoftLimitDirection.kReverse, false);
    // winchMotor.setSoftLimit(SoftLimitDirection.kForward,
    // ElevatorConstants.winchMax);
    // winchMotor.setSoftLimit(SoftLimitDirection.kReverse,
    // (float) winchEncoder.getPosition());

    gripperMotor = new CANSparkMax(ElevatorConstants.gripperPort,
        MotorType.kBrushless);
    gripperEncoder = gripperMotor.getEncoder();
  }

  public void setWinchSpeed(double speed) {
    winchMotor.set(speed);
  }

  public void stopWinch() {
    winchMotor.set(0);
  }

  @Log
  public double getWinchPos() {
    return winchEncoder.getPosition();
  }

  public void resetWinchPos() {
    winchEncoder.setPosition(0);
  }

  public void raiseElevator() {
    setWinchSpeed(ElevatorConstants.winchSpeed);
  }

  public void lowerElevator() {
    setWinchSpeed(-ElevatorConstants.winchSpeed);
  }

  // Methods for gripper control
  public void setGripperSpeed(double speed) {
    gripperMotor.set(speed);
  }

  public void stopGripper() {
    gripperMotor.set(0);
  }

  @Log
  public double getGripperPos() {
    return gripperEncoder.getPosition();
  }

  public void resetGripperPos() {
    gripperEncoder.setPosition(0);
  }

  public void pinchGripper() {
    setGripperSpeed(ElevatorConstants.gripperSpeed);
  }

  public void releaseGripper() {
    setGripperSpeed(-ElevatorConstants.gripperSpeed);
  }

}
