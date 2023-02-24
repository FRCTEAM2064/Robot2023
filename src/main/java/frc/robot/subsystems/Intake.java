// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase implements Loggable {
  Compressor intakeCompressor;
  DoubleSolenoid intakeSolenoid;

  private CANSparkMax leftMotor;
  private CANSparkMax rightMotor;

  /** Creates a new Intake. */
  public Intake() {
    intakeCompressor = new Compressor(20, PneumaticsModuleType.REVPH);
    intakeCompressor.enableDigital();

    intakeSolenoid = new DoubleSolenoid(20, PneumaticsModuleType.REVPH, 0, 1);
    intakeSolenoid.set(kOff);

    leftMotor = new CANSparkMax(IntakeConstants.leftMotorPort, MotorType.kBrushless);
    rightMotor = new CANSparkMax(IntakeConstants.rightMotorPort, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void extend() {
    intakeSolenoid.set(kForward);
  }

  public void retract() {
    intakeSolenoid.set(kReverse);
  }

  public void toggle() {
    if (intakeSolenoid.get() == kForward) {
      retract();
    } else {
      extend();
    }
  }

  public void intakeMotors() {
    leftMotor.set(-IntakeConstants.motorSpeed);
    rightMotor.set(IntakeConstants.motorSpeed);
  }

  public void stopMotors() {
    leftMotor.set(0);
    rightMotor.set(0);
  }
}
