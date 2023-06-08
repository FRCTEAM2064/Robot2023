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
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.SwerveSubsystem;

public class Balance extends CommandBase {
  private final SwerveSubsystem swerveSubsystem;
  private final LEDs leds;

  /** Creates a new Balance. */
  public Balance(SwerveSubsystem swerveSubsystem, LEDs leds) {
    this.swerveSubsystem = swerveSubsystem;
    this.leds = leds;

    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    leds.setPattern("balance");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double pitch = swerveSubsystem.getPitch();
    double roll = swerveSubsystem.getRoll();

    double pitchDiff = (1.39 - pitch);
    double rollDiff = (1.48 - roll);

    if (pitchDiff < 2 && rollDiff < 2) {
      leds.setPattern("rainbow");
    }

    double xSpeed = 3 * pitchDiff / 100;
    double ySpeed = 3 * rollDiff / 100;

    // System.out.println(roll);

    // 3. calculate actual speeds to send to bot
    double maxSpeed = DriveConstants.kTeleDriveMaxSpeedMetersPerSecond / 3;
    xSpeed = Math.copySign(Math.min(Math.abs(xSpeed), maxSpeed), xSpeed);
    ySpeed = Math.copySign(Math.min(Math.abs(ySpeed), maxSpeed), -ySpeed);

    // 4. Construct desired chassis speeds
    ChassisSpeeds chassisSpeeds;
    chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, 0);

    // 5. Convert chassis speeds to individual module states
    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    // 6. Output each module states to wheels
    swerveSubsystem.setModuleStates(moduleStates);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // double pitch = swerveSubsystem.getPitch();
    // double roll = swerveSubsystem.getRoll();

    // double pitchDiff = (1.39 - pitch);
    // double rollDiff = (1.48 - roll);
    return false;
  }
}
