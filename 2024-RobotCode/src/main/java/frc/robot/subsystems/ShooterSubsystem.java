// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

  private final CANSparkMax shootermotor;
  private final CANSparkMax feedermotor;
  
  private final RelativeEncoder shooterencoder;
  private final RelativeEncoder feederencoder;

  private final SparkPIDController pid;

  public ShooterSubsystem() {
    shootermotor = new CANSparkMax(36, MotorType.kBrushless);
    shootermotor.setIdleMode(IdleMode.kCoast);

    feedermotor = new CANSparkMax(36, MotorType.kBrushless);
    feedermotor.setIdleMode(IdleMode.kBrake);
    
    shooterencoder = shootermotor.getEncoder();
    feederencoder = feedermotor.getEncoder();

    pid = shootermotor.getPIDController();
    pid.setP(1);
  }

  public void shooterSpin(double sspeed) {
    shootermotor.set(sspeed);
  }
  
  public void shooterStop() {
    shootermotor.set(0);
  }

  public void feederSpin(double fspeed) {
    feedermotor.set(fspeed);
  }

  public void feederStop() {
    feedermotor.set(0);
  }

  public double getShooterEncoder() {
    return shooterencoder.getPosition();
  }

  public double getFeederEncoder() {
    return feederencoder.getPosition();
  }
  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter Encoder Position", getShooterEncoder());
    SmartDashboard.putNumber("Feeder Encoder Position", getFeederEncoder());
  }
}
