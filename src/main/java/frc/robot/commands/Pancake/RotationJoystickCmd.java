// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Pancake;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PancakeConstants;
import frc.robot.subsystems.Pancake;
import frc.robot.subsystems.Intake;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class RotationJoystickCmd extends CommandBase implements Loggable {
  private final Pancake pancakeSubsystem;
  private final Supplier<Integer> pxnPOVfunction;

  /** Creates a new RotationJoystickCmd. */
  public RotationJoystickCmd(Pancake pancakeSubsystem, Supplier<Integer> pxnPOVFunction){
    // Use addRequirements() here to declare subsystem dependencies.
    this.pancakeSubsystem = pancakeSubsystem;
    this.pxnPOVfunction = pxnPOVFunction;
    addRequirements(pancakeSubsystem);

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
      pancakeSubsystem.stopRotation();
      // pancakeSubsystem.stopTilt();
    }
    Integer val = pxnPOVfunction.get();
    boolean neg = val >= 180;
    if (val == 90 || val == 270) {
      pancakeSubsystem.setRotationSpeed((neg ? 1 : -1) * PancakeConstants.rotationSpeed);
    } else if (val == 0 || val == 180) {
      // pancakeSubsystem.setTiltSpeed((neg ? 1 : -1) * PancakeConstants.tiltSpeed);

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pancakeSubsystem.stopRotation();
    // pancakeSubsystem.stopTilt();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
