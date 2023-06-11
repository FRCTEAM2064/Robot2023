package frc.robot;

import java.util.HashMap;

import javax.management.InstanceAlreadyExistsException;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.Constants.*;
import frc.robot.Constants.OIConstants.pxnButtons;
import frc.robot.commands.Elevator.*;
import frc.robot.commands.Intake.StartIntake;
import frc.robot.commands.Intake.StopIntake;
import frc.robot.commands.Swerve.*;
import frc.robot.subsystems.*;
import io.github.oblarg.oblog.Loggable;

public class RobotContainer implements Loggable {

        private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
        private final Intake intakeSubsystem = new Intake();
        private final Elevator elevatorSubsystem = new Elevator();
        private final LEDs leds = new LEDs();

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
                                () -> driverTurnJoystick.getRawButton(1), leds));
                configureButtonBindings();
                elevatorSubsystem.setDefaultCommand(
                                new ElevatorJoystickCmd(elevatorSubsystem, leds, () -> pxnController.getPOV()));

                SmartDashboard.putData("Buttons/Raise Level 2", new RaiseElevator(elevatorSubsystem, leds, 2));
                SmartDashboard.putData("Buttons/Raise Level 1", new RaiseElevator(elevatorSubsystem, leds, 1));
                SmartDashboard.putData("Buttons/Raise Ground Level", new RaiseElevator(elevatorSubsystem, leds, 0));
                SmartDashboard.putData("Buttons/Lower", new LowerElevator(elevatorSubsystem, leds));
                SmartDashboard.putData("Buttons/Start Intake", new InstantCommand(() -> intakeSubsystem.clean()));
                SmartDashboard.putData("Buttons/Reverse Intake", new InstantCommand(() -> intakeSubsystem.reverse()));
                SmartDashboard.putData("Buttons/Stop Intake", new InstantCommand(() -> intakeSubsystem.stopMotors()));
                SmartDashboard.putData("Buttons/Ready", new ReadyElevator(elevatorSubsystem, leds));
                SmartDashboard.putData("Buttons/Release", new SequentialCommandGroup(new InstantCommand(() -> elevatorSubsystem.setVoltageLimit(80)), new ReleaseGripper(elevatorSubsystem)));
                SmartDashboard.putData("Buttons/Suck", new PinchGripper(elevatorSubsystem));
        }

        public void teleopInit() {

        }

        private void configureButtonBindings() {
                new JoystickButton(driverTurnJoystick, 2)
                                .onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading(), swerveSubsystem));
                new JoystickButton(driverJoystick, 1)
                                .whileTrue(new StartIntake(intakeSubsystem))
                                .onFalse(new StopIntake(intakeSubsystem));
                new JoystickButton(driverJoystick, 8).whileTrue(new Balance(swerveSubsystem, leds));

                new JoystickButton(pxnController, pxnButtons.L1)
                                .whileTrue(new RaiseElevator(elevatorSubsystem, leds, 2));
                new JoystickButton(pxnController, pxnButtons.X)
                                .whileTrue(new RaiseElevator(elevatorSubsystem, leds, 1));
                new JoystickButton(pxnController, pxnButtons.Y)
                                .whileTrue(new RaiseElevator(elevatorSubsystem, leds, 0));
                new JoystickButton(pxnController, pxnButtons.R1).onTrue(new DropAndLower(elevatorSubsystem, leds));

                new JoystickButton(pxnController, pxnButtons.B)
                                .whileTrue(new InstantCommand(() -> intakeSubsystem.reverse())).
                                onFalse(new InstantCommand(() -> intakeSubsystem.stopMotors()));

        }

        public Command getAutonomousCommand(String pathName) {
                HashMap<String, Command> eventMap = new HashMap<String, Command>();
                eventMap.put("drop_intake", new InstantCommand(() -> intakeSubsystem.extend()));
                eventMap.put("raise_intake", new InstantCommand(() -> intakeSubsystem.retract()));
                eventMap.put("start_intake", new InstantCommand(() -> intakeSubsystem.intakeMotors()));
                eventMap.put("stop_intake", new InstantCommand(() -> intakeSubsystem.stopMotors()));
                eventMap.put("eject",
                                new SequentialCommandGroup(
                                                new InstantCommand(() -> intakeSubsystem.reverse(), intakeSubsystem),
                                                new WaitCommand(1), new InstantCommand(
                                                                () -> intakeSubsystem.stopIntake(), intakeSubsystem)));
                eventMap.put("balance", new Balance(swerveSubsystem, leds));
                eventMap.put("elevator_high", new RaiseElevator(elevatorSubsystem, leds, 2));
                eventMap.put("elevator_mid", new RaiseElevator(elevatorSubsystem, leds, 1));
                eventMap.put("elevator_low", new RaiseElevator(elevatorSubsystem, leds, 0));
                eventMap.put("gripper_cube", new InstantCommand(() -> elevatorSubsystem.pinchGripper()).withTimeout(2));
                eventMap.put("drop_and_lower", new DropAndLower(elevatorSubsystem, leds));

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
                return new InstantCommand(() -> swerveSubsystem.zeroHeading()).andThen(builder.fullAuto(path));
        }
}