package frc.robot.commands;

import java.util.function.Supplier;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class VisionandIntake extends Command {

    private final SwerveSubsystem swerveSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final ShooterSubsystem shooterSubsystem;
    private final Supplier<PhotonPipelineResult> visionInfo;
    private PIDController xSpdController, ySpdController, turningSpdController;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    private double xSpeed, ySpeed;
    private PhotonPipelineResult visionResult;
    private int targetLostCounter;
    private int pickupcounts;

    enum State {
        LINE_UP_NOTE,
        DRIVE_FORWARD_INTAKE,
        END
    }

    private State state;
    /**
     * This command writes swerveModuleStates to a photonvision target.
     * @param swerveSubsystem Requirement parameter
     * @param visionInfo PhotonPipelineResult class, input the result
     * @param finishOnTargetLoss IF true, command stops on target loss if LOST and NOT regained 
     * for 20 scheduler cycles. (400 ms) For every cycle there is a target, the counter loses 1.
     */
    public VisionandIntake(SwerveSubsystem swerveSubsystem, IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem, Supplier<PhotonPipelineResult> visionInfo, boolean finishOnTargetLoss) {
        this.swerveSubsystem = swerveSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.shooterSubsystem = shooterSubsystem;

        this.visionInfo = visionInfo;
        xSpdController = new PIDController(0.011, 0, 0);
        xSpdController.setTolerance(2);
        ySpdController = new PIDController(0.013, 0, 0);
        ySpdController.setTolerance(2.5);
        turningSpdController = new PIDController(0.01, 0, 0);
        turningSpdController.setTolerance(2.5);
        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);

        targetLostCounter = 0;
        addRequirements(swerveSubsystem);
        
    }

    @Override
    public void initialize() {
        state = State.LINE_UP_NOTE;
        pickupcounts = 0;
        xSpdController = new PIDController(0.011, 0, 0);
        xSpdController.setTolerance(2);
        ySpdController = new PIDController(0.013, 0, 0);
        ySpdController.setTolerance(2.5);
        turningSpdController = new PIDController(0.01, 0, 0);
        turningSpdController.setTolerance(2.5);

    }

    @Override
    public void execute() {

        switch (state){
            case LINE_UP_NOTE:
            // State transitions
            if (xSpdController.atSetpoint() && ySpdController.atSetpoint()) {
                state = State.DRIVE_FORWARD_INTAKE;
                pickupcounts = 0;
            }
            break;
            case DRIVE_FORWARD_INTAKE:
            // State transitions
            if (pickupcounts >= 750 / 20 || !shooterSubsystem.isBeamBreakTripped()) {
                state = State.END;
            }
            break;
            case END:
                break;
            default:
                break;
        }

        switch (state) {
            case LINE_UP_NOTE:
            // get vision info
            visionResult = visionInfo.get();
            double yaw;
            double pitch;
            double skew;
            if (visionResult.hasTargets() == true) {
                    PhotonTrackedTarget target = visionResult.getBestTarget();
                    yaw = target.getYaw();
                    pitch = target.getPitch();
                    skew = target.getSkew();
                    xSpeed = -xSpdController.calculate(pitch, -14);
                    ySpeed = -ySpdController.calculate(yaw, 0);
                    skew = turningSpdController.calculate(skew, 0);
                    targetLostCounter = targetLostCounter > 0 ? (targetLostCounter - 1) : 0;
            } else {
                // if no target, all speeds are ZERO.
                xSpeed = 0;
                ySpeed = 0;
                skew = 0;
                targetLostCounter++;
            }
            break;
            case DRIVE_FORWARD_INTAKE:
                // Take action
                intakeSubsystem.spinIntake(0.5);
                shooterSubsystem.spinFeeder(0.5);
                shooterSubsystem.spinShooter(-25);
                xSpeed = pickupcounts >= 250 / 20 ? 0 : 2.1;
                ySpeed = 0;
                pickupcounts++;
            break;
            case END:
                xSpeed = 0;
                ySpeed = 0;
                intakeSubsystem.spinIntake(0);
                shooterSubsystem.spinFeeder(0);
                intakeSubsystem.stopIntake();
                shooterSubsystem.stopFeeder();
            break;
            default:
                break;
        }

        

        // 1. Get real-time joystick inputs
        
        double turningSpeed = 0;
        // 3. Make the driving smoother
        xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond / 2;
        ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond / 2;
        turningSpeed = turningLimiter.calculate(turningSpeed)
                * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond / 2;

        // 4. Construct desired chassis speeds
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        // ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0.3, 0);
        SmartDashboard.putNumber("visionxspeed", xSpeed);
        SmartDashboard.putNumber("visionyspeed", ySpeed);
        SmartDashboard.putNumber("visionturningspeed", turningSpeed);
        // 5. Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        SmartDashboard.putString("testBRD", moduleStates[3].toString());
        // 6. Output each module states to wheels
        swerveSubsystem.setModuleStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() { 
        return state == State.END;
    }
}
