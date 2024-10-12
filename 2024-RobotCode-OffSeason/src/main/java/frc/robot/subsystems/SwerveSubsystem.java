package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class SwerveSubsystem extends SubsystemBase {
    

    private final SwerveModule frontLeft = new SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
            "FL-Offset",
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule frontRight = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
            "FR-Offset",
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

    private final SwerveModule backLeft = new SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
            "BL-Offset",
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule backRight = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderPort,
            "BR-Offset",
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed);

    private final AHRS gyro = new AHRS(SPI.Port.kMXP);
    
    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics,
            new Rotation2d(0),
            new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
              }); 

    public SwerveSubsystem() {

        new Thread(() -> {
            try {
                Thread.sleep(1000);
                resetHeading();
            } catch (Exception e) {
            }
        }).start();
        // Configure AutoBuilder last
        AutoBuilder.configureHolonomic(
                this::getPose, // Robot pose supplier
                this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                        5.121, // Max module speed, in m/s
                        0.33, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() != DriverStation.Alliance.Blue;
                }
                return false; //Defaults to blue
                },
                this // Reference to this subsystem to set requirements
        );
    }
    
    public ChassisSpeeds getRobotRelativeSpeeds(){
        return DriveConstants.kDriveKinematics.toChassisSpeeds(
            frontLeft.getState(),
            frontRight.getState(),
            backLeft.getState(),
            backRight.getState()
            );
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        SwerveModuleState[] states = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
        this.setModuleStates(states);
    } 

    public void resetHeading() {
        gyro.reset();
        gyro.setAngleAdjustment(getHeading());
    }

    public void redoOffsets () {
        Preferences.setDouble("FL-Offset", 0);
        Preferences.setDouble("BL-Offset", 0);
        Preferences.setDouble("FR-Offset", 0);
        Preferences.setDouble("BR-Offset", 0);

        Preferences.setDouble("FL-Offset", Math.IEEEremainder(-frontLeft.getAbsoluteEncoderRad(), 2 * Math.PI));
        Preferences.setDouble("BL-Offset", Math.IEEEremainder(-backLeft.getAbsoluteEncoderRad(), 2 * Math.PI));
        Preferences.setDouble("FR-Offset", Math.IEEEremainder(-frontRight.getAbsoluteEncoderRad(), 2 * Math.PI));
        Preferences.setDouble("BR-Offset", Math.IEEEremainder(-backRight.getAbsoluteEncoderRad(), 2 * Math.PI));
    }

    public double getHeading() {
        return Math.IEEEremainder(-gyro.getAngle(), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }


    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(
            getRotation2d(), 
            new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
            },
            pose);
    }

    public void resetEncoders() {
        frontLeft.resetEncoders();
        frontRight.resetEncoders();
        backLeft.resetEncoders();
        backRight.resetEncoders();
    }

    @Override
    public void periodic() {
        // Update the odometer to accurately track robot position.
        odometer.update(
            getRotation2d(), 
            new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
        
            }
        );

        SmartDashboard.putNumber("FL ABS Calculated Radians" , frontLeft.getAbsoluteEncoderRad());
        SmartDashboard.putNumber("FR ABS Calculated Radians" , frontRight.getAbsoluteEncoderRad());
        SmartDashboard.putNumber("BL ABS Calculated Radians" , backLeft.getAbsoluteEncoderRad());
        SmartDashboard.putNumber("BR ABS Calculated Radians" , backRight.getAbsoluteEncoderRad());
        SmartDashboard.putNumber("FL Drive Position" , frontLeft.getDrivePosition());
        SmartDashboard.putNumber("FR Drive Position" , frontRight.getDrivePosition());
        SmartDashboard.putNumber("BL Drive Position" , backLeft.getDrivePosition());
        SmartDashboard.putNumber("BR Drive Position" , backRight.getDrivePosition());
        SmartDashboard.putNumber("FL Turning Position" , frontLeft.getTurningPosition());
        SmartDashboard.putNumber("FR Turning Position" , frontRight.getTurningPosition());
        SmartDashboard.putNumber("BL Turning Position" , backLeft.getTurningPosition());
        SmartDashboard.putNumber("BR Turning Position" , backRight.getTurningPosition());
        SmartDashboard.putNumber("GetHeading", getHeading());
        
    }

    
    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

}
