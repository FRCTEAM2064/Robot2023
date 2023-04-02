// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.SoftLimitDirection;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;

import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase implements Loggable {
  Compressor intakeCompressor;
  DoubleSolenoid intakeSolenoid;

  private CANSparkMax motor;

  /** Creates a new Intake. */
  public Intake() {
    intakeCompressor = new Compressor(20, PneumaticsModuleType.REVPH);
    intakeCompressor.enableDigital();

    intakeSolenoid = new DoubleSolenoid(20, PneumaticsModuleType.REVPH, 8, 9);
    intakeSolenoid.set(kOff);

    motor = new CANSparkMax(IntakeConstants.leftMotorPort, MotorType.kBrushless);

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
    motor.set(IntakeConstants.motorSpeed);
  }

  public void clean() {
    motor.set(IntakeConstants.motorSpeed * 0.2);
  }

  public void stopMotors() {
    motor.set(0);
  }

  public void stopIntake() {
    motor.set(0);
  }

  public void reverse() {
    motor.set(-1);
  }

  public void hold() {
    motor.set(IntakeConstants.motorSpeed * 0.2);
  }

}
