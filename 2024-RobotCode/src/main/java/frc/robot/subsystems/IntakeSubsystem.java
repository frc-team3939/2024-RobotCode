// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */

  private final CANSparkMax intakeMotor;
  private final SparkPIDController pid;

  private final RelativeEncoder intakeCoder;
  private final DigitalInput intakeLimitSwitch;

  public IntakeSubsystem() {
    //Change ID once robot is wired
    intakeMotor = new CANSparkMax(51, MotorType.kBrushless);
    // IdleMode is brake vs coast. Brake stops when it stops recieving power, coast will let it coast.
    intakeMotor.setIdleMode(IdleMode.kBrake);

    // This assignment gets the encoder from the motor object defined earlier. A RelativeEncoder is an object created with each CANSparkMax controller.
    intakeCoder = intakeMotor.getEncoder();
    // A PID controller is a Proportion, Integral, and Derivative controller. It outputs numbers based on the input given, the parameters of the PID, and the "setpoint" or target value.
    pid = intakeMotor.getPIDController();
    pid.setP(1);
    // A digital input is the slots 0-9 on the RoboRIO in the "DIO" area. You plug in limit switches into here normally. Essentially, this declaration points to the number 9 slot on the DIO. 
    intakeLimitSwitch = new DigitalInput(9);
  }

  /**
   * Simple function to spin the intake motor at the parameter speed. 
   * @param speed Speed between -1.0 and 1.0.
   */
  public void spinIntake(double speed) {
    // set(speed) is the simple way to set speed for a SparkMAX. It differs slightly from a Talon - see another subsystem for that.
    intakeMotor.set(speed);
  }
  
  /**
   * Simple function to stop the intake. It is good to have this as opposed to running spinIntake(0), because it ensures there is no error.
   */
  public void stopIntake() {
    intakeMotor.set(0);
  }

  /**
   * Returns the intake encoder position. The spinning wheels do have a position - the encoder is located inside the NEO.
   * @return The double encoder position.
   */
  // public double getIntakeEncoder() {
  //   return intakeCoder.getPosition();
  // }

  public double getIntakeSpeed() {
    return intakeCoder.getVelocity();
  }

  /**
   * Returns a boolean on whether the limit switches were tripped in the intake mechanism. Notice how the value is negated - this is a simple code change due to wiring necessities.
   * It is much easier to change code like this than wires.
   * @return Boolean for whether limit is tripped. True is tripped, false is not.
   */
  public boolean isIntakeLimitSwitchTripped() {
    return !intakeLimitSwitch.get();
  }
  
  /**
   * Periodic function standard to all subsystems.
   */
  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Intake Limit Switch", isIntakeLimitSwitchTripped());
    SmartDashboard.putNumber("Intake Speed", getIntakeSpeed());
    // SmartDashboard.putNumber("Intake Encoder", getIntakeEncoder());
  }
}
