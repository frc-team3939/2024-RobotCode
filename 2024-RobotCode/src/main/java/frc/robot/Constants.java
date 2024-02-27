package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {

    public static final class ModuleConstants {
        
        //Check AndyMark for specifications on the following 3 variables
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kDriveMotorGearRatio = 1 / 6.75;
        public static final double kTurningMotorGearRatio = 1 / 12.8;
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        public static final double kPTurning = 0.5;
    }

    public static final class DriveConstants {

        public static final double kTrackWidth = Units.inchesToMeters(18.5);
        // Distance between the center of the right and left wheels in inches
        public static final double kWheelBase = Units.inchesToMeters(18.5);
        // Distance between the center of the front and back wheels in inches
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2));

        //Plug into any SparkMax with a USB-C cable and use the utility on the desktop to check the port IDs of each motor 
        public static final int kFrontLeftDriveMotorPort = 23;   //FL
        public static final int kBackLeftDriveMotorPort = 43;    //BL
        public static final int kFrontRightDriveMotorPort = 27;  //FR
        public static final int kBackRightDriveMotorPort = 41;   //BR

        public static final int kFrontLeftTurningMotorPort = 28;     //FL
        public static final int kBackLeftTurningMotorPort = 24;      //BL
        public static final int kFrontRightTurningMotorPort = 42;    //FR
        public static final int kBackRightTurningMotorPort = 40;     //BR

        public static final boolean kFrontLeftTurningEncoderReversed = true;
        public static final boolean kBackLeftTurningEncoderReversed = true;
        public static final boolean kFrontRightTurningEncoderReversed = true;
        public static final boolean kBackRightTurningEncoderReversed = true;

        public static final boolean kFrontLeftDriveEncoderReversed = true;
        public static final boolean kBackLeftDriveEncoderReversed = false;
        public static final boolean kFrontRightDriveEncoderReversed = true;
        public static final boolean kBackRightDriveEncoderReversed = false;

        //I think we need to use the analog ports on the robo rio for these???
        public static final int kFrontLeftDriveAbsoluteEncoderPort = 2;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 5;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 3;
        public static final int kBackRightDriveAbsoluteEncoderPort = 4;

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = true;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = true;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = true;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = true;

        //***** We need to change these numbers once we get swerve up and working *****
        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 2.858;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 0.331;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = -3.117;
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = -2.787;

        //Check Andymark for the max speed of the swerve modules IN METERS PER SECOND
        public static final double kPhysicalMaxSpeedMetersPerSecond = 4.27;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 1;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
                kPhysicalMaxAngularSpeedRadiansPerSecond / 1;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 5;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 5;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 4;
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
        public static final int kTopHalfButtonBoardPort = 1;
        public static final int kBottomHalfButtonBoardPort = 2;

        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 4;
        // public static final int kLeftTriggerAxis = 3;
        // public static final int kRightTriggerAxis = 4;
        public static void main(String[] args) {
            
        }
        public static final int kDriverFieldOrientedButtonIdx = 1;

        public static final double kDeadband = 0.15;
    }
}
