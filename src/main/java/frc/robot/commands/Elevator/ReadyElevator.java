// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ReadyElevator extends PIDCommand {
    private final Elevator elevatorSubsystem;

    /** Creates a new ReadyElevator. */
    public ReadyElevator(Elevator elevatorSubsystem) {
        super(
                // The controller that the command will use
                new PIDController(ElevatorConstants.winchP, ElevatorConstants.winchI, ElevatorConstants.winchD),
                // This should return the measurement
                () -> elevatorSubsystem.getWinchPos(),
                // This should return the setpoint (can also be a constant)
                ElevatorConstants.winchReady,
                // This uses the output
                output -> {
                    System.out.println("lowering: " + output);
                    elevatorSubsystem.setWinchSpeed(output);
                });
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(elevatorSubsystem);
        // Configure additional PID options by calling `getController` here.
        this.elevatorSubsystem = elevatorSubsystem;
        getController().setTolerance(1);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return getController().atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        this.elevatorSubsystem.stopWinch();
    }
}
