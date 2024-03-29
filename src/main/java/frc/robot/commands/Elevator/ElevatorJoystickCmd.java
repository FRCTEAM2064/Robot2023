// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Elevator;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Pancake;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class ElevatorJoystickCmd extends CommandBase implements Loggable {
  private final Elevator elevatorSubsystem;
  private final Supplier<Integer> pxnPOVfunction;
  private final LEDs leds;

  /** Creates a new ElevatorJoystickCmd. */
  public ElevatorJoystickCmd(Elevator elevatorSubsystem, LEDs leds, Supplier<Integer> pxnPOVFunction){
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevatorSubsystem = elevatorSubsystem;
    this.pxnPOVfunction = pxnPOVFunction;
    this.leds = leds;
    addRequirements(elevatorSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("POV", pxnPOVfunction.get());
    if (pxnPOVfunction.get() == -1) {
      if (leds.getPattern() == "elevatorUp" || leds.getPattern() == "elevatorDown") {
        leds.setPattern("pattern");
      }
      elevatorSubsystem.stopWinch();
    }
    Integer val = pxnPOVfunction.get();
    if (val == 0) {
      leds.setPattern("elevatorUp");
      elevatorSubsystem.raiseElevator();
    } else if (val == 180) {
      leds.setPattern("elevatorDown");
      elevatorSubsystem.lowerElevator();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.stopWinch();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
