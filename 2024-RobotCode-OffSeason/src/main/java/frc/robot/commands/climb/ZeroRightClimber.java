// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

public class ZeroRightClimber extends Command {
  private final ClimbSubsystem climbSubsystem;
  public ZeroRightClimber(ClimbSubsystem subsystem) {
    climbSubsystem = subsystem;
    addRequirements(climbSubsystem);
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climbSubsystem.moveRightClimber(-1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climbSubsystem.zeroRightClimbEncoder();
    climbSubsystem.stopArms();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return climbSubsystem.isRightLimitSwitchTripped();
  }
}
