// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PivotSubsystem extends SubsystemBase {

  private final CANSparkMax pivotmotor;
  
  private final RelativeEncoder encoder;

  public PivotSubsystem() {
    pivotmotor = new CANSparkMax(51, MotorType.kBrushless);
    pivotmotor.setIdleMode(IdleMode.kBrake);
    
    encoder = pivotmotor.getEncoder();
  }

  public void zeroPivotEncoders() {
    encoder.setPosition(0);
  }

  public void moveArmPosition (double movePosition) {
    encoder.setPosition(movePosition);  
  }

  public boolean isRedSide() {
    if (getPivotPosition() < 0)
      return true;
    else 
      return false;
  }

  public void stopPivot () {
    pivotmotor.set(0);
  }

  public double getPivotPosition() {
    return encoder.getPosition();
  }

  public void movePivot (double moveSpeed) {
    pivotmotor.set(moveSpeed);
  }

  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Pivot Encoder Position", getPivotPosition());
  }
}
