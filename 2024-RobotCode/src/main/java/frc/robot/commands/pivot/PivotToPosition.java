// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PivotSubsystem;

public class PivotToPosition extends Command {
  /** Creates a new ZeroPivotEncoders. */
  double targ_position;
  private final PivotSubsystem pivotsubsystem;
  public PivotToPosition(PivotSubsystem subsystem, double position) {
    pivotsubsystem = subsystem;
    targ_position = position;
    addRequirements(pivotsubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pivotsubsystem.movePivot(targ_position);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pivotsubsystem.movePivot(targ_position);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pivotsubsystem.movePivot(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}