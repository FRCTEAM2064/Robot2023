package frc.robot;

import java.util.HashMap;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.Constants.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import io.github.oblarg.oblog.Loggable;

public class RobotContainer implements Loggable {

        private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

        // private final Vision visionSubsystem = new Vision();

        private final Joystick driverJoystick = new Joystick(OIConstants.kDriverControllerPort);
        private final Joystick driverTurnJoystick = new Joystick(OIConstants.kDriverTurnControllerPort);

        public RobotContainer() {
                swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                                swerveSubsystem,
                                () -> driverJoystick.getRawAxis(0), // X axis
                                () -> driverJoystick.getRawAxis(1), // y axis
                                () -> driverTurnJoystick.getRawAxis(0), // turning speed
                                () -> !driverJoystick.getRawButton(1)));

                configureButtonBindings();
        }

        private void configureButtonBindings() {
                new JoystickButton(driverJoystick, 1).onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));
        }

        public Command getAutonomousCommand(String pathName, HashMap<String, Command> eventMap) {
                PathPlannerTrajectory path = PathPlanner.loadPath(pathName, AutoConstants.kMaxSpeedMetersPerSecond,
                                AutoConstants.kMaxAccelerationMetersPerSecondSquared);
                SwerveAutoBuilder builder = new SwerveAutoBuilder(
                                swerveSubsystem::getPose,
                                swerveSubsystem::resetOdometry,
                                DriveConstants.kDriveKinematics,
                                new PIDConstants(AutoConstants.kPXController, 0, 0),
                                new PIDConstants(AutoConstants.kPThetaController, 0, 0),
                                swerveSubsystem::setModuleStates,
                                eventMap,
                                true,
                                swerveSubsystem);
                return builder.fullAuto(path);
        }
}