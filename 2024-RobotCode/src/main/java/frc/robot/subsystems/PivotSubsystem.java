// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PivotSubsystem extends SubsystemBase {

  private final TalonSRX pivotmotor;
  private final VictorSPX slavemotor;

  private final DutyCycleEncoder absencoder;

  public PivotSubsystem() {
    
    pivotmotor = new TalonSRX(22);
    slavemotor = new VictorSPX(35);
    absencoder = new DutyCycleEncoder(0);

    pivotmotor.setNeutralMode(NeutralMode.Brake);
    slavemotor.setNeutralMode(NeutralMode.Brake);

    pivotmotor.setInverted(false);
    slavemotor.setInverted(true);

    resetEncoders();
  }

  public void zeroPivotEncoders() {
    absencoder.reset();
  }

  public void SetPowerLeft(double percent) {
    pivotmotor.set(ControlMode.PercentOutput, percent);
  }

  public void SetPowerRight(double percent) {
    slavemotor.set(ControlMode.PercentOutput, percent);
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
    double absoluteEncoderOffsetRad = -2.59;
    double angle = absencoder.getAbsolutePosition();
    angle = angle * 2 * Math.PI;
    return -(angle + absoluteEncoderOffsetRad);
  }

  public double getPGPivotPositionRad() {
    double angle = pivotmotor.getSelectedSensorPosition();
    angle = angle * 1 * Math.PI / 497;
    return angle;
  }

  public void movePivot (double movePosition) {
    pivotmotor.set(ControlMode.Position , movePosition);
    slavemotor.set(ControlMode.Position , -movePosition);
  }

  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("ABS Pivot Encoder Position", getABSPivotPositionRad());
    SmartDashboard.putNumber("PG Pivot Encoder Position", getPGPivotPositionRad());
  }
}
