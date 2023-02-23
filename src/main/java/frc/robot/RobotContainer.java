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
import frc.robot.commands.Elevator.LowerElevator;
import frc.robot.commands.Elevator.RaiseElevator;
import frc.robot.commands.Elevator.*;
import frc.robot.commands.Pancake.*;
import frc.robot.commands.Swerve.*;
import frc.robot.commands.Vision.*;
import frc.robot.commands.Pancake.RotationJoystickCmd;
import frc.robot.commands.Vision.TurnToBestTag;
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
                                .onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));
                new JoystickButton(driverJoystick, 8).whileTrue(new Balance(swerveSubsystem));
                new JoystickButton(pxnController, pxnButtons.X)
                                .onTrue(new InstantCommand(() -> intakeSubsystem.toggle()));
                new JoystickButton(pxnController, pxnButtons.Y).onTrue(new TurnToBestTag(swerveSubsystem, limeLight));

                new JoystickButton(pxnController, pxnButtons.A)
                                .onTrue(new RaiseElevator(elevatorSubsystem)
                                                .alongWith(new PrintCommand("going up")));
                new JoystickButton(pxnController, pxnButtons.B)
                                .onTrue(new LowerElevator(elevatorSubsystem)
                                                .alongWith(new PrintCommand("going down")))
                                .onFalse(new InstantCommand(elevatorSubsystem::stopWinch));
                // Control for Gripper
                new JoystickButton(pxnController, pxnButtons.Y)
                                .onTrue(new InstantCommand(
                                                () -> elevatorSubsystem.setGripperSpeed(ElevatorConstants.gripperSpeed))
                                                .until(() -> elevatorSubsystem
                                                                .getGripperPos() == ElevatorConstants.gripperMin)
                                                .alongWith(new PrintCommand("Pinching gripper")));
                new JoystickButton(pxnController, pxnButtons.R1)
                                .onTrue(new InstantCommand(
                                                () -> elevatorSubsystem.setGripperSpeed(ElevatorConstants.gripperSpeed))
                                                .until(() -> elevatorSubsystem
                                                                .getGripperPos() == ElevatorConstants.gripperMax)
                                                .alongWith(new PrintCommand("Releasing gripper")));
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