// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PivotSubsystem extends SubsystemBase {

  private final TalonSRX pivotmotor;
  private final VictorSPX followermotor;

  private final DutyCycleEncoder absencoder;

  private double desired_power;
  private double position_target;

  public PivotSubsystem() {
    
    pivotmotor = new TalonSRX(22);
    followermotor = new VictorSPX(35);
    absencoder = new DutyCycleEncoder(0);

    pivotmotor.setNeutralMode(NeutralMode.Coast);
    followermotor.setNeutralMode(NeutralMode.Coast);

    pivotmotor.setInverted(false);
    followermotor.setInverted(true);

    desired_power = 0;

    pivotmotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder,
                                            0,1000);
    pivotmotor.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20, 1000);
    pivotmotor.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, 1000);
    pivotmotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, 1000);

    pivotmotor.config_kP(0, 12);
    pivotmotor.config_kI(0, 0);
    pivotmotor.config_kD(0, 0);
    pivotmotor.config_kF(0, 1);

    resetEncoders();

    followermotor.follow(pivotmotor);

    // pivotmotor.configSelectedFeedbackSensor(FeedbackDevice.Analog);

    

    
  }

  public void zeroPivotEncoders() {
    absencoder.reset();
  }

  public void SetPowerLeft(double percent) {
    // pivotmotor.set(ControlMode.PercentOutput, percent);
  }

  public void SetPowerRight(double percent) {
    // followermotor.set(ControlMode.PercentOutput, percent);
  }

  public void SetDesiredPower(double percent){
    desired_power = percent;
  }

  public boolean isRedSide() {
    if (getABSPivotPositionRad() < 0)
      return true;
    else 
      return false;
  }

  public void resetEncoders() {
    pivotmotor.setSelectedSensorPosition(getABSPivotPositionRad() * 497 / (1 * Math.PI));
  }

  public double getABSPivotPositionRad() {
    double absoluteEncoderOffsetRad = -5.17;
    double angle = absencoder.getAbsolutePosition();
    angle = angle * 4 * Math.PI;
    return -(angle + absoluteEncoderOffsetRad);
  }

  public double getPGPivotPositionRad() {
    double angle = pivotmotor.getSelectedSensorPosition();
    angle = angle * 1 * Math.PI / 497;
    return angle;
  }

  public void movePivot (double movePosition) {
    position_target = movePosition;
    pivotmotor.set(ControlMode.Position, movePosition);
  }

  
  @Override
  public void periodic() {
    // double ff_val = desired_power;
    // double gravity_factor = -0.3*Math.sin(getABSPivotPositionRad());
    // ff_val += gravity_factor;
    // // pivotmotor.set(ControlMode.PercentOutput, ff_val);
    // pivotmotor.set(ControlMode.Position,position_target,DemandType.ArbitraryFeedForward,ff_val);
    //followermotor.set(ControlMode.PercentOutput, cmd_val);
    // if (Math.abs(getPGPivotPositionRad()) > 0.01 ){ // Prevent divide by zero
    //   SmartDashboard.putNumber("Pivot/ABS-INC Ratio", getABSPivotPositionRad()/getPGPivotPositionRad());
    // }
    SmartDashboard.putNumber("Pivot/ABS Encoder Position", getABSPivotPositionRad());
    SmartDashboard.putNumber("Pivot/PG Encoder Position", getPGPivotPositionRad());
    SmartDashboard.putNumber("Pivot/Target (encoder counts)", position_target);
  }

}
