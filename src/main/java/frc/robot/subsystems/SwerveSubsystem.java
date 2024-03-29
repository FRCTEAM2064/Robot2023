package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class SwerveSubsystem extends SubsystemBase implements Loggable {
    private final SwerveModule frontLeft = new SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule frontRight = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

    // private final SwerveModule backLeft = new SwerveModule(
    // DriveConstants.kBackLeftDriveMotorPort,
    // DriveConstants.kBackLeftTurningMotorPort,
    // DriveConstants.kBackLeftDriveEncoderReversed,
    // DriveConstants.kBackLeftTurningEncoderReversed,
    // DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
    // DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
    // DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule backRight = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderPort,
            DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed);

    private final AHRS gyro = new AHRS(SPI.Port.kMXP);
    SwerveModulePosition[] positions = {
            frontLeft.getPosition(), frontRight.getPosition(), backRight.getPosition(), backRight.getPosition() };
    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics,
            new Rotation2d(0), positions);

    public SwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
    }

    public void zeroHeading() {
        gyro.reset();
    }

    public double getHeading() {
        return gyro.getAngle();
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading()).plus(Rotation2d.fromDegrees(90));
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        SwerveModulePosition[] positions = {
                frontLeft.getPosition(), frontRight.getPosition(), backRight.getPosition(), backRight.getPosition() };
        odometer.update(getRotation2d(), positions);
        odometer.resetPosition(getRotation2d(), positions, pose);
    }

    @Override
    public void periodic() {
        SwerveModulePosition[] positions = {
                frontLeft.getPosition(), frontRight.getPosition(), backRight.getPosition(), backRight.getPosition() };
        odometer.update(getRotation2d(), positions);
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());

        double[] arr = getAbsolutePositions();
        SmartDashboard.putNumber("Front Left Turn Encoder", arr[0]);
        SmartDashboard.putNumber("Front Right Turn Encoder", arr[1]);
        SmartDashboard.putNumber("Back Right Turn Encoder", arr[2]);

        SmartDashboard.putNumber("Pitch", gyro.getPitch());
        SmartDashboard.putNumber("Roll", gyro.getRoll());
        SmartDashboard.putNumber("Yaw", gyro.getYaw());
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        // backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        // backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    public double[] getAbsolutePositions() {
        return new double[] {
                frontLeft.getAbsoluteEncoderRad(),
                frontRight.getAbsoluteEncoderRad(),
                // backLeft.getAbsoluteEncoderRad(),
                backRight.getAbsoluteEncoderRad()
        };
    }

    public double getPitch() {
        return gyro.getPitch();
    }

    public double getRoll() {
        return gyro.getRoll();
    }
}