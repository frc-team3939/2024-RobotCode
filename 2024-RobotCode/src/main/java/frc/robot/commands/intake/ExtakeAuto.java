// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ExtakeAuto extends Command {
  private final IntakeSubsystem intakeSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private double speed;
  private int i;
  public ExtakeAuto(IntakeSubsystem isubsystem, ShooterSubsystem ssubsystem, double speed) {
    intakeSubsystem = isubsystem;
    shooterSubsystem = ssubsystem;
    this.speed = speed;
    addRequirements(intakeSubsystem);
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    i = 0;
    intakeSubsystem.spinIntake(speed);
    shooterSubsystem.spinFeeder(speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeSubsystem.spinIntake(speed);
    shooterSubsystem.spinFeeder(speed);
    i++;
  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.stopIntake();
    shooterSubsystem.stopFeeder();
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (i >= 3) {
      return true;
    }
    else {
    return false;
    }
  }
}
