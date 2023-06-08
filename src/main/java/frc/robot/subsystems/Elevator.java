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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase implements Loggable {
  private CANSparkMax winchRightMotor;
  private CANSparkMax winchLeftMotor;
  
  private RelativeEncoder winchRightEncoder;
  private RelativeEncoder winchLeftEncoder;

  private CANSparkMax gripperMotor;

  /** Creates a new Elevator. */
  public Elevator() {
    winchRightMotor = new CANSparkMax(ElevatorConstants.winchRightPort, MotorType.kBrushless);
    winchLeftMotor = new CANSparkMax(ElevatorConstants.winchLeftPort, MotorType.kBrushless);
    winchRightEncoder = winchRightMotor.getEncoder();
    winchLeftEncoder = winchLeftMotor.getEncoder();



    winchLeftMotor.follow(winchRightMotor, true);
    winchRightMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    winchRightMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    winchRightMotor.setSoftLimit(SoftLimitDirection.kForward,
        ElevatorConstants.winchMax);
        winchRightMotor.setSoftLimit(SoftLimitDirection.kReverse,
        ElevatorConstants.winchMin);

    gripperMotor = new CANSparkMax(ElevatorConstants.gripperPort,
        MotorType.kBrushless);
  }


  @Override
  public void periodic() {
    SmartDashboard.putData(this);
  }

  public void setWinchSpeed(double speed) {
    winchRightMotor.set(speed);
  }

  public void stopWinch() {
    winchRightMotor.set(0);
  }

  @Log
  public double getWinchPos() {
    return winchRightEncoder.getPosition();
  }

  public void resetWinchPos() {
    winchRightEncoder.setPosition(0);
    winchLeftEncoder.setPosition(0);
  }

  public void raiseElevator() {
    setWinchSpeed(ElevatorConstants.winchSpeed);
  }

  public void lowerElevator() {
    setWinchSpeed(-ElevatorConstants.winchSpeed);
  }

  // Methods for gripper control
  public void setGripperSpeed(double speed) {
    System.out.println("Set gripper speed: " + speed);
    gripperMotor.set(speed);
  }

  public void stopGripper() {
    System.out.println("Stopping gripper");
    gripperMotor.set(0);
  }

  public void pinchGripper() {
    setGripperSpeed(-ElevatorConstants.gripperSpeed);
  }

  public void releaseGripper() {
    setGripperSpeed(ElevatorConstants.gripperSpeed);
  }

}
