package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public final class Constants implements Loggable {

    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kDriveMotorGearRatio = 1 / 8.31;
        public static final double kTurningMotorGearRatio = 1 / 18.0;
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        public static final double kPTurning = 0.5;
    }

    public static final class DriveConstants implements Loggable {
        // distances are set

        public static final double kTrackWidth = Units.inchesToMeters(23);
        // Distance between right and left wheels
        public static final double kWheelBase = Units.inchesToMeters(23);
        // Distance between front and back wheels
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2));

        // CAN ports are set

        public static final int kFrontLeftDriveMotorPort = 15;
        public static final int kBackLeftDriveMotorPort = 16;
        public static final int kFrontRightDriveMotorPort = 18;
        public static final int kBackRightDriveMotorPort = 8;

        public static final int kFrontLeftTurningMotorPort = 35;
        public static final int kBackLeftTurningMotorPort = 7;
        public static final int kFrontRightTurningMotorPort = 17;
        public static final int kBackRightTurningMotorPort = 22;

        public static final boolean kFrontLeftTurningEncoderReversed = true;
        public static final boolean kBackLeftTurningEncoderReversed = true;
        public static final boolean kFrontRightTurningEncoderReversed = true;
        public static final boolean kBackRightTurningEncoderReversed = true;

        public static final boolean kFrontLeftDriveEncoderReversed = true;
        public static final boolean kBackLeftDriveEncoderReversed = false;
        public static final boolean kFrontRightDriveEncoderReversed = false;
        public static final boolean kBackRightDriveEncoderReversed = true;

        public static final int kFrontLeftDriveAbsoluteEncoderPort = 1;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 2;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 0;
        public static final int kBackRightDriveAbsoluteEncoderPort = 3;

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 4.8;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 1.77;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 6.2;
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 0.6;

        public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 2;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond
                / 4;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 4;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;

        public static double balanceRollP = 0;
        public static double balanceRollI = 0;
        public static double balanceRollD = 0;
        public static double balancePitchP = 0;
        public static double balancePitchI = 0;
        public static double balancePitchD = 0;

        @Config
        public void setRollPID(double p, double i, double d) {
            balanceRollP = p;
            balanceRollI = i;
            balanceRollD = d;
        }

        @Config
        public void setPitchPID(double p, double i, double d) {
            balancePitchP = p;
            balancePitchI = i;
            balancePitchD = d;
        }

    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 8;
        public static final double kMaxAngularSpeedRadiansPerSecond = //
                DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
        public static final double kPXController = 1.5;
        public static final double kPYController = 1.5;
        public static final double kPThetaController = 3;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kDriverTurnControllerPort = 1;
        public static final int kPXNControllerPort = 2;

        public static final class pxnButtons {
            public static final int L1 = 5;
            // public static final int L2 = 7; don't use this
            public static final int R1 = 6;
            // public static final int R2 = 8; don't use this
            public static final int A = 2;
            public static final int B = 3;
            public static final int X = 1;
            public static final int Y = 4;
            public static final int Share = 9;
            public static final int Options = 10;
            public static final int L3 = 11;
            public static final int R3 = 12;
        }

        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 4;
        public static final int kDriverFieldOrientedButtonIdx = 1;

        public static final double kDeadband = 0.05;
    }

    public static final class PancakeConstants {
        public static final int rotationPort = 10;
        public static final double rotationSpeed = 0.1;
        
    }

    public static final class ElevatorConstants implements Loggable {
        public static final int winchPort = 12;
        public static final double winchSpeed = 0.1;
        public static final int gripperPort = 37;
        public static final double gripperSpeed = 0.2;

        public static final float winchMax = 130;
        public static final float winchMin = 0;
        public static final double gripperMax = 0;
        public static final double gripperMin = 0.85;

        @Log
        public static double winchP = 0.4;
        public static double gripperP = 1;
        @Log
        public static double winchI = 0;
        public static double gripperI = 0;
        @Log
        public static double winchD = 0;
        public static double gripperD = 0;

        @Config
        public void setGripperPID(double p, double i, double d) {
            gripperP = p;
            gripperI = i;
            gripperD = d;
        }
    }

    public static final class IntakeConstants {
        public static final int leftMotorPort = 33;
        public static final int rightMotorPort = 23;
        public static final int tiltPort = 19;

        public static final double motorSpeed = 0.29;
        public static final double tiltSpeed = 0.2;

        public static final float maxTiltValue = 2; // TODO: add value
        public static final float minTiltValue = -2; // TODO: add value
    }
}