package frc.robot;

import java.util.HashMap;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

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
import frc.robot.commands.Vision.*;
import frc.robot.subsystems.*;
import io.github.oblarg.oblog.Loggable;

public class RobotContainer implements Loggable {

        private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
        private final LimeLight limeLight = new LimeLight();
        private final Intake intakeSubsystem = new Intake();
        private final Elevator elevatorSubsystem = new Elevator();
        private final LEDs leds = new LEDs();

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
                                () -> driverTurnJoystick.getRawButton(1), leds));
                elevatorSubsystem.setDefaultCommand(
                                new ElevatorJoystickCmd(elevatorSubsystem, () -> pxnController.getPOV()));
                configureButtonBindings();

                SmartDashboard.putData("Raise Elevator High", new RaiseElevator(elevatorSubsystem, true));
                SmartDashboard.putData("Raise Elevator Low", new RaiseElevator(elevatorSubsystem, false));
                SmartDashboard.putData("Lower Elevator", new LowerElevator(elevatorSubsystem));
                SmartDashboard.putData("Start Intake", new InstantCommand(() -> intakeSubsystem.clean()));
                SmartDashboard.putData("Reverse Intake", new InstantCommand(() -> intakeSubsystem.reverse()));
                SmartDashboard.putData("Stop Intake", new InstantCommand(() -> intakeSubsystem.stopMotors()));
                SmartDashboard.putData("Ready Elevator", new ReadyElevator(elevatorSubsystem));
                SmartDashboard.putData("Release Gripper", new ReleaseGripper(elevatorSubsystem));
                SmartDashboard.putData("Gripper Cone", new PinchGripper(elevatorSubsystem,
                                false));
                SmartDashboard.putData("Gripper Cube", new PinchGripper(elevatorSubsystem,
                                true));
        }

        private void configureButtonBindings() {
                new JoystickButton(driverTurnJoystick, 2)
                                .onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading(), swerveSubsystem));

                new JoystickButton(driverJoystick, 8).whileTrue(new Balance(swerveSubsystem, leds));
                new JoystickButton(driverJoystick, 1)
                                .whileTrue(new StartIntake(intakeSubsystem))
                                .onFalse(new StopIntake(intakeSubsystem));

                new JoystickButton(pxnController, pxnButtons.X)
                                .onTrue(new PinchGripper(elevatorSubsystem, true));
                new JoystickButton(pxnController, pxnButtons.Y)
                                .onTrue(new PinchGripper(elevatorSubsystem, false));
                new JoystickButton(pxnController, pxnButtons.L1)
                                .onTrue(new ReleaseGripper(elevatorSubsystem).andThen(new WaitCommand(1))
                                                .andThen(new LowerElevator(elevatorSubsystem)));

                new JoystickButton(pxnController, pxnButtons.A)
                                .onTrue(new RaiseElevator(elevatorSubsystem, true));
                new JoystickButton(pxnController, pxnButtons.B)
                                .onTrue(new RaiseElevator(elevatorSubsystem, false));

                new JoystickButton(pxnController, pxnButtons.R1)
                                .whileTrue(new InstantCommand(() -> intakeSubsystem.reverse(), intakeSubsystem));

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
                                                new WaitCommand(5), new InstantCommand(
                                                                () -> intakeSubsystem.stopIntake(), intakeSubsystem)));
                eventMap.put("balance", new Balance(swerveSubsystem, leds));

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