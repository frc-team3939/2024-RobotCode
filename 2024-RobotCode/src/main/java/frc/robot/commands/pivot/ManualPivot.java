// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.pivot;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PivotSubsystem;

public class ManualPivot extends Command {
  private final PivotSubsystem pivotSubsystem;
  Supplier<Double> powerFunction;
  public ManualPivot(PivotSubsystem subsystem, Supplier<Double> spdFunction) {
    pivotSubsystem = subsystem;
    this.powerFunction = spdFunction;
    addRequirements(pivotSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double power = powerFunction.get();
    // pivotSubsystem.SetPowerLeft(power);
    // pivotSubsystem.SetPowerRight(power);
    pivotSubsystem.SetDesiredPower(power);
    SmartDashboard.putNumber("Pivot Power", power);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pivotSubsystem.SetPowerLeft(0);
    pivotSubsystem.SetPowerRight(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
