package frc.robot;

import java.util.List;

import javax.swing.ButtonGroup;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.PS4Controller.Axis;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.intake.SpinIntake;
import frc.robot.commands.pivot.PivotToPositionPID;
import frc.robot.commands.ResetHeading;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
 
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final PivotSubsystem pivotSubsystem = new PivotSubsystem();
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

    private final Joystick driverJoytick = new Joystick(OIConstants.kDriverControllerPort);
    private final PS4Controller driverPS4Controller = new PS4Controller(OIConstants.kDriverControllerPort);

    Trigger button1 = new JoystickButton(driverJoytick, 1);
    Trigger button2 = new JoystickButton(driverJoytick, 2);
    Trigger button3 = new JoystickButton(driverJoytick, 3);
    Trigger button4 = new JoystickButton(driverJoytick, 4);
    Trigger button5 = new JoystickButton(driverJoytick, 5);
    Trigger button6 = new JoystickButton(driverJoytick, 6);
    Trigger button7 = new JoystickButton(driverJoytick, 7);
    Trigger button8 = new JoystickButton(driverJoytick, 8);
    Trigger button9 = new JoystickButton(driverJoytick, 9);
    Trigger button10 = new JoystickButton(driverJoytick, 10);
    Trigger button11 = new JoystickButton(driverJoytick, 11);
    Trigger button12 = new JoystickButton(driverJoytick, 12);

    public RobotContainer() {
        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                swerveSubsystem,
                () -> driverJoytick.getRawAxis(OIConstants.kDriverYAxis),
                () -> -driverJoytick.getRawAxis(OIConstants.kDriverXAxis),
                () -> driverJoytick.getRawAxis(OIConstants.kDriverRotAxis),
                () -> !driverJoytick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        //button1.whileTrue(new SpinIntake(intakeSubsystem, 1));
        button1.onTrue(new ResetHeading(swerveSubsystem));
        button5.whileTrue(new SpinIntake(intakeSubsystem, shooterSubsystem, 0.33));
        button6.whileTrue(new SpinIntake(intakeSubsystem, shooterSubsystem, -0.33));
        button3.onTrue(new PivotToPositionPID(pivotSubsystem, 0));
        new JoystickButton(driverPS4Controller, PS4Controller.Button.kCross.value).toggleOnTrue(new ResetHeading(swerveSubsystem));
    }

    public Command getAutonomousCommand() {
        // 1. Create trajectory settings
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        .setKinematics(DriveConstants.kDriveKinematics);

        // 2. Generate trajectory
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(
                        new Translation2d(1, 0),
                        new Translation2d(1, -1)),
                new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
                trajectoryConfig);

        // 3. Define PID controllers for tracking trajectory
        PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
        PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // 4. Construct command to follow trajectory
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                trajectory,
                swerveSubsystem::getPose,
                DriveConstants.kDriveKinematics,
                xController,
                yController,
                thetaController,
                swerveSubsystem::setModuleStates,
                swerveSubsystem);

        // 5. Add some init and wrap-up, and return everything
        return new SequentialCommandGroup(
                new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
                swerveControllerCommand,
                new InstantCommand(() -> swerveSubsystem.stopModules()));
    }
}
