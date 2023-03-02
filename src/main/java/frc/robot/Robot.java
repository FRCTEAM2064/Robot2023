// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.nio.file.FileSystem;
import java.util.HashMap;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.utils.Log;

import frc.robot.Constants.*;

import io.github.oblarg.oblog.Logger;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private RobotContainer m_robotContainer;

    private static String kDefaultAuto;
    private static String[] paths;
    private String m_autoSelected;
    private final SendableChooser<String> m_chooser = new SendableChooser<>();

    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        // Instantiate our RobotContainer. This will perform all our button bindings,
        // and put our
        // autonomous chooser on the dashboard.
        m_robotContainer = new RobotContainer();

        paths = this.getPaths();
        if (paths.length > 0) {
            kDefaultAuto = paths[0];

            for (String path : paths) {
                Log.info("Loading path: " + path);
                m_chooser.addOption(path, path);
            }
            m_chooser.setDefaultOption(kDefaultAuto, kDefaultAuto);
        }
        SmartDashboard.putData("Autonomous Selection", m_chooser);

        Logger.configureLoggingAndConfig(this, false);

        CameraServer.startAutomaticCapture();
    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for
     * items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and
     * test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled
        // commands, running already-scheduled commands, removing finished or
        // interrupted commands,
        // and running subsystem periodic() methods. This must be called from the
        // robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
        Logger.updateEntries();

    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    /**
     * This autonomous runs the autonomous command selected by your
     * {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        HashMap<String, Command> eventMap = new HashMap<String, Command>();
        // eventMap.put("intake", new IntakeCommand())

        m_autoSelected = m_chooser.getSelected();

        try {
            m_autonomousCommand = m_robotContainer.getAutonomousCommand(m_autoSelected, eventMap);
        } catch (Exception e) {
            Log.error("Error loading autonomous command: " + e.getMessage());
            e.printStackTrace();
        }

        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        } else {
            System.out.println("No auto selected or failed to run");
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {
    }

    private String[] getPaths() {
        // read the autos folder and get all the classes
        // return them in an array

        File dir = new File(Filesystem.getDeployDirectory() + "/pathplanner");
        File[] directoryListing = dir.listFiles();
        if (directoryListing == null) {
            return new String[0];
        }

        String[] paths = new String[directoryListing.length];
        for (int i = 0; i < directoryListing.length; i++) {
            String name = directoryListing[i].getName();
            if (name.endsWith(".path")) {
                paths[i] = name.substring(0, name.length() - 5);
                Log.info("Found path: " + paths[i]);
            }
        }
        return paths;
    }
}