package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {

    private final CANSparkMax climbmotorLeft;
    private final CANSparkMax climbmotorRight;

    private final RelativeEncoder climbencoderLeft;
    private final RelativeEncoder climbencoderRight;

    private final DigitalInput limitswitchLeft;
    private final DigitalInput limitswitchRight;

    private final SparkPIDController climbmotorLeftPID;
    private final SparkPIDController climbmotorRightPID;

    public ClimbSubsystem() {

        //Change ID values when robot is wired
        climbmotorLeft = new CANSparkMax(52, MotorType.kBrushless);
        climbmotorLeft.setIdleMode(IdleMode.kBrake);

        climbmotorRight = new CANSparkMax(55, MotorType.kBrushless);
        climbmotorRight.setIdleMode(IdleMode.kBrake);

        climbencoderLeft = climbmotorLeft.getEncoder();
        climbencoderRight = climbmotorRight.getEncoder();

        climbmotorLeftPID = climbmotorLeft.getPIDController();
        climbmotorRightPID = climbmotorLeft.getPIDController();

        //Change channel values when robot is wired
        limitswitchLeft = new DigitalInput(1);
        limitswitchRight = new DigitalInput(2);

        climbmotorLeft.setIdleMode(IdleMode.kBrake);
        climbmotorLeft.setSoftLimit(SoftLimitDirection.kForward, 108.5f);

        climbmotorLeftPID.setOutputRange(-1, 1);
        climbmotorLeftPID.setP(5e-5);
        climbmotorLeftPID.setI(1e-6);
        climbmotorLeftPID.setD(0);
        climbmotorLeftPID.setIZone(0);
        climbmotorLeftPID.setFF(0.000156);
        climbmotorLeftPID.setSmartMotionMaxVelocity(4200, 0);
        climbmotorLeftPID.setSmartMotionMinOutputVelocity(0, 0);
        climbmotorLeftPID.setSmartMotionMaxAccel(3250, 0);
        climbmotorLeftPID.setSmartMotionAllowedClosedLoopError(0.01, 0);

        climbmotorRight.setIdleMode(IdleMode.kBrake);
        //climbmotorRight.setSoftLimit(SoftLimitDirection.kForward, 350);

        climbmotorRightPID.setOutputRange(-1, 1);
        climbmotorRightPID.setP(5e-5);
        climbmotorRightPID.setI(1e-6);
        climbmotorRightPID.setD(0);
        climbmotorRightPID.setIZone(0);
        climbmotorRightPID.setFF(0.000156);
        climbmotorRightPID.setSmartMotionMaxVelocity(4200, 0);
        climbmotorRightPID.setSmartMotionMinOutputVelocity(0, 0);
        climbmotorRightPID.setSmartMotionMaxAccel(3250, 0);
        climbmotorRightPID.setSmartMotionAllowedClosedLoopError(0.01, 0);
  }


    public void zeroClimbEncoders() {
        climbencoderLeft.setPosition(0);
        climbencoderRight.setPosition(0);
    }

    public void zeroLeftClimbEncoder() {
        climbencoderLeft.setPosition(0);
    }

    public void zeroRightClimbEncoder() {
        climbencoderRight.setPosition(0);
    }


    public void moveClimber(double speed) {
        climbmotorLeft.set(-speed);
        climbmotorRight.set(speed);

    }

    public void moveLeftClimber(double speed) {
        climbmotorLeft.set(speed);
    }

    public void moveRightClimber(double speed) {
        climbmotorRight.set(speed);
    }

    public void stopArms () {
        climbmotorLeft.stopMotor();
        climbmotorRight.stopMotor();
    }

    public void stopLeftArm () {
        climbmotorLeft.set(0);
    }

    public void stopRightArm () {
        climbmotorRight.set(0);
    }


    public boolean isLeftLimitSwitchTripped() {
        return limitswitchLeft.get();
    }

    public boolean isRightLimitSwitchTripped() {
        return limitswitchRight.get();
    }


    public double getLeftArmPosition() {
        return climbencoderLeft.getPosition();
    }

    public double getRightArmPosition() {
        return climbencoderRight.getPosition();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Left Arm Position", getLeftArmPosition());
        SmartDashboard.putNumber("Right Arm Position", getRightArmPosition());
        SmartDashboard.putBoolean("Left Limit Switch", isLeftLimitSwitchTripped());
        SmartDashboard.putBoolean("Right Limit Switch", isRightLimitSwitchTripped());
    }



}
