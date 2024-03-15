// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class ShootTrap extends Command {
  // Creates a new ShootCommand
  double ss;
  double fs;
  double i;
  private final ShooterSubsystem shootersubsystem;
  private final PivotSubsystem pivotsubsystem;
  public ShootTrap(ShooterSubsystem subsystem, PivotSubsystem psubsystem, double ss) {
    shootersubsystem = subsystem;
    pivotsubsystem = psubsystem;
    this.ss = ss;
    addRequirements(shootersubsystem);


  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    i = 0;
    if (pivotsubsystem.isRedSide() == true) {
      shootersubsystem.RedspinShooterAmp(ss);
    }
    else {
      shootersubsystem.BluespinShooterAmp(ss);
    }

    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    i = i + 1;
    //ss = 70;//(SmartDashboard.getNumber("Speed", 0)); //Delete when speed is decided on
    shootersubsystem.spinShooterTrap(ss);
    if (pivotsubsystem.isRedSide() == true) {
      shootersubsystem.RedspinShooterAmp(ss);
    }
    else {
      shootersubsystem.BluespinShooterAmp(ss);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shootersubsystem.stopShooter();
    shootersubsystem.stopFeeder();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shootersubsystem.isBeamBreakTripped();
  }
}
