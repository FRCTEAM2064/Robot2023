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
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class ElevatorJoystickCmd extends CommandBase implements Loggable {
  private final Elevator elevatorSubsystem;
  private final Supplier<Integer> pxnPOVfunction;

  /** Creates a new ElevatorJoystickCmd. */
  public ElevatorJoystickCmd(Elevator elevatorSubsystem, Supplier<Integer> pxnPOVFunction) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevatorSubsystem = elevatorSubsystem;
    this.pxnPOVfunction = pxnPOVFunction;
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
      elevatorSubsystem.stopWinch();
    }
    Integer val = pxnPOVfunction.get();
    boolean neg = val >= 180;
    if (val == 0) {
      elevatorSubsystem.raiseElevator();
    } else if (val == 180) {
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
