// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Vision;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TurnToBestTag extends PIDCommand {
  /** Creates a new TurnToBestTag. */
  public TurnToBestTag(SwerveSubsystem swerveSubsystem, Vision visionSubsystem) {
    super(
        // The controller that the command will use
        new PIDController(0.3, 0, 0),
        // This should return the measurement
        () -> visionSubsystem.getTargetX(),
        // This should return the setpoint (can also be a constant)
        0,
        // This uses the output
        output -> {
          SlewRateLimiter turningLimiter = new SlewRateLimiter(
              DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
          double turningSpeed = turningLimiter.calculate(output)
              * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

          // 4. Construct desired chassis speeds
          ChassisSpeeds chassisSpeeds;
          // Relative to robot
          chassisSpeeds = new ChassisSpeeds(0, 0, turningSpeed);

          // 5. Convert chassis speeds to individual module states
          SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

          // 6. Output each module states to wheels
          swerveSubsystem.setModuleStates(moduleStates);
        });
    addRequirements(swerveSubsystem);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
