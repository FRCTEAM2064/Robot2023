// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.SwerveSubsystem;

public class MoveToBestTag extends CommandBase {
  private final PIDController xController;
  private final PIDController yController;
  private final PIDController rotationController;

  private final LimeLight visionSubsystem;
  private final SwerveSubsystem swerveSubsystem;

  public MoveToBestTag(SwerveSubsystem swerveSubsystem, LimeLight visionSubsystem) {
    xController = new PIDController(0.1, 0, 0);
    yController = new PIDController(0.1, 0, 0);
    rotationController = new PIDController(0.1, 0, 0);

    xController.setSetpoint(0);
    yController.setSetpoint(0);
    rotationController.setSetpoint(0);

    xController.setTolerance(0.05);

    addRequirements(swerveSubsystem);
    this.swerveSubsystem = swerveSubsystem;
    this.visionSubsystem = visionSubsystem;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed = xController.calculate(visionSubsystem.getdegRotationToTarget());

    double maxMovementSpeed = DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
    double actualXSpeed = Math.copySign(Math.min(Math.abs(xSpeed),
        maxMovementSpeed), xSpeed);

    // 4. Construct desired chassis speeds
    ChassisSpeeds chassisSpeeds;
    // Relative to robot
    chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(actualXSpeed, 0,
        0, swerveSubsystem.getRotation2d());

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
    return xController.atSetpoint();
  }
}
