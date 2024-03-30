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
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

  private final CANSparkMax Bshootermotor;
  private final CANSparkMax Tshootermotor;
  private final CANSparkMax feedermotor;
  
  private final RelativeEncoder Bshooterencoder;
  private final RelativeEncoder Tshooterencoder;
  private final RelativeEncoder feederencoder;

  private final SparkPIDController Bpid;
  private final SparkPIDController Tpid;
  private final SparkPIDController Fpid;

  private final DigitalInput lowerbeambreak;
  private final DigitalInput upperbeambreak;
  

  public ShooterSubsystem() {

    Preferences.initDouble("Feeder P", 0.0002);
    Preferences.initDouble("Top Shooter P", 0.0002);
    Preferences.initDouble("Bottom Shooter P", 0.0002);

    Preferences.initDouble("Trap Speed", 0.75);


    Bshootermotor = new CANSparkMax(53, MotorType.kBrushless);
    Bshootermotor.setIdleMode(IdleMode.kCoast);

    Tshootermotor = new CANSparkMax(57, MotorType.kBrushless);
    Tshootermotor.setIdleMode(IdleMode.kCoast);

    feedermotor = new CANSparkMax(58, MotorType.kBrushless);
    feedermotor.setIdleMode(IdleMode.kBrake);
    
    Bshootermotor.enableVoltageCompensation(11.5);
    Tshootermotor.enableVoltageCompensation(11.5);

    Bshooterencoder = Bshootermotor.getEncoder();
    Tshooterencoder = Tshootermotor.getEncoder();
    feederencoder = feedermotor.getEncoder();

    Bpid = Bshootermotor.getPIDController();
    Bpid.setP(Preferences.getDouble("Bottom Shooter P", 0.0002));

    Tpid = Tshootermotor.getPIDController();
    Tpid.setP(Preferences.getDouble("Top Shooter P", 0.0002));

    Fpid = feedermotor.getPIDController();
    Fpid.setP(Preferences.getDouble("Feeder P", 0.0002));

    lowerbeambreak = new DigitalInput(3);
    upperbeambreak = new DigitalInput(2);//change
  }

  public void spinShooter(double sspeed) {
    Bshootermotor.set(-sspeed);
    Tshootermotor.set(sspeed);
    // Bpid.setP(Preferences.getDouble("Bottom Shooter P", 0.0002));
    // Bpid.setReference(-sspeed * 6000, CANSparkMax.ControlType.kVelocity);
    // Tpid.setP(Preferences.getDouble("Top Shooter P", 0.0002));
    // Tpid.setReference(sspeed * 6000, CANSparkMax.ControlType.kVelocity);
  }

  public void spinShooterAmp(double sspeed) {
    Bshootermotor.set(-sspeed);
    Tshootermotor.set(sspeed - 0.15);
    // Bpid.setP(Preferences.getDouble("Bottom Shooter P", 0.0002));
    // Bpid.setReference(-sspeed * 5676, CANSparkMax.ControlType.kVelocity);
    // Tpid.setP(Preferences.getDouble("Top Shooter P", 0.0002));
    // Tpid.setReference((sspeed - 0.15) * 5676, CANSparkMax.ControlType.kVelocity);
  }

  public void spinShooterTrap(double sspeed) {
    Bshootermotor.set(-sspeed);
    Tshootermotor.set(sspeed - 0.85);
  }
  
  public void stopShooter() {
    Bshootermotor.set(0);
    Tshootermotor.set(0);
  }

  public void spinFeeder(double fspeed) {
    Fpid.setP(Preferences.getDouble("Feeder P", 0.0002));
    Fpid.setReference(fspeed * 5676, CANSparkMax.ControlType.kVelocity);
    // feedermotor.set(fspeed);
  }

  public void stopFeeder() {
    feedermotor.set(0);
  }

  public double getBottomShooterVelocity() {
    return Bshooterencoder.getVelocity();
  }

  public double getTopShooterVelocity() {
    return Tshooterencoder.getVelocity();
  }
  
  public double getFeederVelocity() {
    return feederencoder.getVelocity();
  }

  public boolean isLowerBeamBreakTripped() {
    return lowerbeambreak.get();
  }

  public boolean isUpperBeamBreakTripped() {
    return upperbeambreak.get();
  }

  public void setShooterBrake() {
    Bshootermotor.setIdleMode(IdleMode.kBrake);
    Tshootermotor.setIdleMode(IdleMode.kBrake);
  }

  public void setShooterCoast() {
    Bshootermotor.setIdleMode(IdleMode.kCoast);
    Tshootermotor.setIdleMode(IdleMode.kCoast);
  }
  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Bottom Flywheel Velocity", getBottomShooterVelocity());
    SmartDashboard.putNumber("Top Flywheel Velocity", getTopShooterVelocity());
    SmartDashboard.putNumber("Feeder Velocity", getFeederVelocity());
    SmartDashboard.putBoolean("Lower Beam Break State", !lowerbeambreak.get());
    SmartDashboard.putBoolean("Upper Beam Break State", !upperbeambreak.get());
  }
}
