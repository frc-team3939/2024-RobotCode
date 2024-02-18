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

  private final CANSparkMax Yshootermotor;
  private final CANSparkMax Sshootermotor;
  private final CANSparkMax feedermotor;
  
  private final RelativeEncoder Yshooterencoder;
  private final RelativeEncoder Sshooterencoder;
  private final RelativeEncoder feederencoder;

  private final SparkPIDController Ypid;
  private final SparkPIDController Spid;

  public ShooterSubsystem() {
    Yshootermotor = new CANSparkMax(53, MotorType.kBrushless);
    Yshootermotor.setIdleMode(IdleMode.kCoast);

    Sshootermotor = new CANSparkMax(57, MotorType.kBrushless);
    Sshootermotor.setIdleMode(IdleMode.kCoast);

    feedermotor = new CANSparkMax(58, MotorType.kBrushless);
    feedermotor.setIdleMode(IdleMode.kBrake);
    
    Yshootermotor.enableVoltageCompensation(11.5);
    Sshootermotor.enableVoltageCompensation(11.5);

    Yshooterencoder = Yshootermotor.getEncoder();
    Sshooterencoder = Sshootermotor.getEncoder();
    feederencoder = feedermotor.getEncoder();

    Ypid = Yshootermotor.getPIDController();
    Ypid.setP(1);

    Spid = Sshootermotor.getPIDController();
    Spid.setP(1);
  }

  public void spinShooter(double sspeed) {
    Yshootermotor.set(sspeed);
    Sshootermotor.set(-sspeed);
  }
  
  public void stopShooter() {
    Yshootermotor.set(0);
    Sshootermotor.set(0);
  }

  public void spinFeeder(double fspeed) {
    feedermotor.set(fspeed);
  }

  public void stopFeeder() {
    feedermotor.set(0);
  }

  public double getYellowShooterEncoder() {
    return Yshooterencoder.getPosition();
  }

  public double getSilverShooterEncoder() {
    return Sshooterencoder.getPosition();
  }

  public double getFeederEncoder() {
    return feederencoder.getPosition();
  }
  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Yellow Shooter Encoder", getYellowShooterEncoder());
    SmartDashboard.putNumber("Silver Shooter Encoder", getSilverShooterEncoder());
    SmartDashboard.putNumber("Feeder Encoder", getFeederEncoder());
  }
}
