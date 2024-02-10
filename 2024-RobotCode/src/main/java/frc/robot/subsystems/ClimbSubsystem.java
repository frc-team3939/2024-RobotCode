package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

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

    public ClimbSubsystem() {

        //Change ID values when robot is wired
        climbmotorLeft = new CANSparkMax(36, MotorType.kBrushless);
        climbmotorLeft.setIdleMode(IdleMode.kBrake);

        climbmotorRight = new CANSparkMax(36, MotorType.kBrushless);
        climbmotorRight.setIdleMode(IdleMode.kBrake);

        climbencoderLeft = climbmotorLeft.getEncoder();
        climbencoderRight = climbmotorRight.getEncoder();

        //Change channel values when robot is wired
        limitswitchLeft = new DigitalInput(8);
        limitswitchRight = new DigitalInput(8);

    }

    public void climberDown(double speed) {
        climbmotorLeft.set(speed);
        climbmotorRight.set(speed);
    }

    public void LeftclimberDown(double speed) {
        climbmotorLeft.set(speed);
    }

    public void RightclimberDown(double speed) {
        climbmotorRight.set(speed);
    }


    public void climberUp() {
        climbmotorLeft.setIdleMode(IdleMode.kCoast);
        climbmotorRight.setIdleMode(IdleMode.kCoast);
    }

    public void LeftclimberUp() {
        climbmotorLeft.setIdleMode(IdleMode.kCoast);
    }

    public void RightclimberUp() {
        climbmotorLeft.setIdleMode(IdleMode.kCoast);
        climbmotorRight.setIdleMode(IdleMode.kCoast);
    }

    public void climberLock() {
        climbmotorLeft.setIdleMode(IdleMode.kBrake);
        climbmotorRight.setIdleMode(IdleMode.kBrake);
    }

    // public void zeroClimbEncoders() {
    //     climbencoderLeft.setPosition(0);
    //     climbencoderRight.setPosition(0);
    // }

    // public void moveClimbPosition(double positionDouble) {
    //     climbencoderLeft.setPosition(positionDouble);
    //     climbencoderRight.setPosition(positionDouble);
    // }

    // public void moveLeftClimbPosition(double positionDouble) {
    //     climbencoderLeft.setPosition(positionDouble);
    // }

    // public void moveRightClimbPosition(double positionDouble) {
    //     climbencoderRight.setPosition(positionDouble);
    // }

    // public void stopArms () {
    //     climbmotorLeft.set(0);
    //     climbmotorRight.set(0);
    // }

    // public void stopLeftArm () {
    //     climbmotorLeft.set(0);
    // }

    // public void stopRightArm () {
    //     climbmotorRight.set(0);
    // }


    public boolean isLeftLimitSwitchTripped() {
        return !limitswitchLeft.get();
    }

    public boolean isRightLimitSwitchTripped() {
        return !limitswitchRight.get();
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
