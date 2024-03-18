// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

  private final CANSparkMax Rshootermotor;
  private final CANSparkMax Bshootermotor;
  private final CANSparkMax feedermotor;
  
  private final RelativeEncoder Rshooterencoder;
  private final RelativeEncoder Bshooterencoder;
  private final RelativeEncoder feederencoder;

  private final SparkPIDController Rpid;
  private final SparkPIDController Bpid;

  private final DigitalInput feeder_beambreak;

  public ShooterSubsystem() {
    Rshootermotor = new CANSparkMax(53, MotorType.kBrushless);
    Rshootermotor.setIdleMode(IdleMode.kCoast);

    Bshootermotor = new CANSparkMax(57, MotorType.kBrushless);
    Bshootermotor.setIdleMode(IdleMode.kCoast);

    feedermotor = new CANSparkMax(58, MotorType.kBrushless);
    feedermotor.setIdleMode(IdleMode.kBrake);
    
    Rshootermotor.enableVoltageCompensation(11.5);
    Bshootermotor.enableVoltageCompensation(11.5);

    Rshooterencoder = Rshootermotor.getEncoder();
    Bshooterencoder = Bshootermotor.getEncoder();
    feederencoder = feedermotor.getEncoder();

    Rpid = Rshootermotor.getPIDController();
    Rpid.setP(1);

    Bpid = Bshootermotor.getPIDController();
    Bpid.setP(1);

    feeder_beambreak = new DigitalInput(3);
  }

  public void spinShooter(double sspeed) {
    Rshootermotor.set(sspeed);
    Bshootermotor.set(-sspeed);
  }

  public void spinShooterAmp(double sspeed) {
    Rshootermotor.set(sspeed);
    Bshootermotor.set(-sspeed + 0.15);
  }

  public void spinShooterTrap(double sspeed) {
    Rshootermotor.set(sspeed);
    Bshootermotor.set(-sspeed + 0.55);
  }
  
  public void stopShooter() {
    Rshootermotor.set(0);
    Bshootermotor.set(0);
  }

  public void spinFeeder(double fspeed) {
    feedermotor.set(fspeed);
  }

  public void stopFeeder() {
    feedermotor.set(0);
  }

  public double getRedShooterVelocity() {
    return Rshooterencoder.getVelocity();
  }

  public double getBlueShooterVelocity() {
    return Bshooterencoder.getVelocity();
  }
  
  public double getFeederVelocity() {
    return feederencoder.getVelocity();
  }

  public boolean isBeamBreakTripped() {
    return feeder_beambreak.get();
  }

  public void setShooterBrake() {
    Rshootermotor.setIdleMode(IdleMode.kBrake);
    Bshootermotor.setIdleMode(IdleMode.kBrake);
  }

  public void setShooterCoast() {
    Rshootermotor.setIdleMode(IdleMode.kCoast);
    Bshootermotor.setIdleMode(IdleMode.kCoast);
  }
  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Red Flywheel Velocity", getRedShooterVelocity());
    SmartDashboard.putNumber("Blue Flywheel Velocity", getBlueShooterVelocity());
    SmartDashboard.putNumber("Feeder Velocity", getFeederVelocity());
    SmartDashboard.putBoolean("Feeder Beam Break State", feeder_beambreak.get());
  }
}
