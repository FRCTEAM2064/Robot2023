package frc.robot;

import java.util.HashMap;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.Constants.*;
import frc.robot.Constants.OIConstants.pxnButtons;
import frc.robot.commands.Elevator.*;
import frc.robot.commands.Intake.StartIntake;
import frc.robot.commands.Intake.StopIntake;
import frc.robot.commands.Pancake.*;
import frc.robot.commands.Swerve.*;
import frc.robot.commands.Vision.*;
import frc.robot.commands.Pancake.RotationJoystickCmd;
import frc.robot.subsystems.*;
import io.github.oblarg.oblog.Loggable;

public class RobotContainer implements Loggable {

        private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
        private final LimeLight limeLight = new LimeLight();
        private final Intake intakeSubsystem = new Intake();
        private final Pancake pancakeSubsystem = new Pancake();
        private final Elevator elevatorSubsystem = new Elevator();

        private final ElevatorConstants constants = new ElevatorConstants();
        private final DriveConstants dConstants = new DriveConstants();

        private final Joystick driverJoystick = new Joystick(OIConstants.kDriverControllerPort);
        private final Joystick driverTurnJoystick = new Joystick(OIConstants.kDriverTurnControllerPort);
        private final Joystick pxnController = new Joystick(OIConstants.kPXNControllerPort);

        public RobotContainer() {
                swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                                swerveSubsystem,
                                () -> driverJoystick.getRawAxis(0), // X axis
                                () -> driverJoystick.getRawAxis(1), // y axis
                                () -> -driverTurnJoystick.getRawAxis(0), // turning speed
                                () -> !driverJoystick.getRawButton(1), // field oriented button
                                () -> driverJoystick.getRawButton(1)));
                pancakeSubsystem.setDefaultCommand(
                                new RotationJoystickCmd(pancakeSubsystem, () -> pxnController.getPOV()));
                configureButtonBindings();
        }

        private void configureButtonBindings() {
                new JoystickButton(driverTurnJoystick, 1)
                                .whileTrue((new MoveToBestTag(swerveSubsystem, limeLight)));

                new JoystickButton(driverJoystick, 8).whileTrue(new Balance(swerveSubsystem));
                new JoystickButton(driverJoystick, 1)
                                .onTrue(new StartIntake(intakeSubsystem))
                                .onFalse(new StopIntake(intakeSubsystem));

                // new JoystickButton(pxnController, pxnButtons.Y).onTrue(new
                // TurnToBestTag(swerveSubsystem, limeLight));
                // new JoystickButton(pxnController, pxnButtons.A)
                // .onTrue(new PinchGripper(elevatorSubsystem, true));
                // new JoystickButton(pxnController, pxnButtons.B)
                // .onTrue(new PinchGripper(elevatorSubsystem, false));
                new JoystickButton(pxnController, pxnButtons.L1)
                                .onTrue(new ReleaseGripper(elevatorSubsystem));

                new JoystickButton(pxnController, pxnButtons.A)
                                .onTrue(new InstantCommand(() -> elevatorSubsystem.setWinchSpeed(0.5))
                                                .alongWith(new PrintCommand("going up")))
                                .onFalse(new InstantCommand(() -> elevatorSubsystem.setWinchSpeed(0)));
                new JoystickButton(pxnController, pxnButtons.B)
                                .onTrue(new InstantCommand(() -> elevatorSubsystem.setWinchSpeed(-0.5))
                                                .alongWith(new PrintCommand("going down")))
                                .onFalse(new InstantCommand(() -> elevatorSubsystem.setWinchSpeed(0)));

        }

        public Command getAutonomousCommand(String pathName) {
                HashMap<String, Command> eventMap = new HashMap<String, Command>();
                eventMap.put("drop_intake", new InstantCommand(() -> intakeSubsystem.extend()));
                eventMap.put("raise_intake", new InstantCommand(() -> intakeSubsystem.retract()));
                eventMap.put("start_intake", new InstantCommand(() -> intakeSubsystem.intakeMotors()));
                eventMap.put("stop_intake", new InstantCommand(() -> intakeSubsystem.stopMotors()));

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