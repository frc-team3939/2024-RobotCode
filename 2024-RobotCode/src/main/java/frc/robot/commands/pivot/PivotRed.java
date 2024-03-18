// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PivotSubsystem;

public class PivotRed extends Command {
  /** Creates a new ZeroPivotEncoders. */
  double targ_position;
  int i;
  boolean RedState;
  private final PivotSubsystem pivotsubsystem;
  public PivotRed(PivotSubsystem subsystem) {
    pivotsubsystem = subsystem;
    addRequirements(pivotsubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    i = 0;
    RedState = pivotsubsystem.isRedSide();
    pivotsubsystem.setPivotCoast(true);
    if (pivotsubsystem.isRedSide() == false) {
      pivotsubsystem.movePivot(0); //Change with testing
    }
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    i = i + 1;
    if (i >= 17) {
      if (RedState == false) {
        pivotsubsystem.movePivot(7); //Change with testing
      }
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pivotsubsystem.setPivotCoast(false);
  }
    
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (i >= 35) {
      return true;
      }
      else {
        return false;
      }
      }
}
