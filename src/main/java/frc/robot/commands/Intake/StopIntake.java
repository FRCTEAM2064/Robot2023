// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

// import javax.management.InstanceAlreadyExistsException;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Treadmill;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class StopIntake extends SequentialCommandGroup {
  /** Creates a new StopIntake. */
  Intake intakeSubsystem;

  public StopIntake(Intake intakeSubsystem, Treadmill treadmillSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new InstantCommand(() -> intakeSubsystem.retract()),
        new InstantCommand(() -> intakeSubsystem.stopIntake()),
        new InstantCommand(() -> intakeSubsystem.stopRollers()),
        new InstantCommand(() -> treadmillSubsystem.stopTreadmill()));

    addRequirements(intakeSubsystem, treadmillSubsystem);
  }
}
