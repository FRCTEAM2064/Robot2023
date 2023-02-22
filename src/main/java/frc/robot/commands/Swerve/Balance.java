// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;

public class Balance extends CommandBase implements Loggable {
  private final SwerveSubsystem swerveSubsystem;

  // private PIDController rollController;
  // private PIDController pitchController;

  /** Creates a new Balance. */
  public Balance(SwerveSubsystem swerveSubsystem) {
    this.swerveSubsystem = swerveSubsystem;

    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // rollController = new PIDController(DriveConstants.balanceRollP,
    // DriveConstants.balanceRollI,
    // DriveConstants.balanceRollD);
    // pitchController = new PIDController(DriveConstants.balancePitchP,
    // DriveConstants.balancePitchI,
    // DriveConstants.balancePitchD);

    // rollController.setSetpoint(-1);
    // pitchController.setSetpoint(-0.3);

    // rollController.setTolerance(4);
    // pitchController.setTolerance(4);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // double xSpeed = pitchController.calculate(swerveSubsystem.getPitch());
    // double ySpeed = rollController.calculate(swerveSubsystem.getRoll());

    // xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
    // ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;

    double pitch = swerveSubsystem.getPitch();
    double roll = swerveSubsystem.getRoll();

    double pitchDiff = (-0.2 - pitch);
    double rollDiff = -(-0.7 - roll);

    double xSpeed = Math.pow(10, -3 / pitchDiff);
    double ySpeed = Math.pow(10, -3 / rollDiff);

    // System.out.println(roll);

    // 3. calculate actual speeds to send to bot
    double maxSpeed = DriveConstants.kTeleDriveMaxSpeedMetersPerSecond / 3;
    xSpeed = Math.copySign(Math.min(Math.abs(xSpeed), maxSpeed), xSpeed);
    ySpeed = Math.copySign(Math.min(Math.abs(ySpeed), maxSpeed), -ySpeed);

    // 4. Construct desired chassis speeds
    ChassisSpeeds chassisSpeeds;
    chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        xSpeed, ySpeed, 0, swerveSubsystem.getRotation2d());

    // 5. Convert chassis speeds to individual module states
    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    // 6. Output each module states to wheels
    swerveSubsystem.setModuleStates(moduleStates);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
