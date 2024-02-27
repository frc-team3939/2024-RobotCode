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
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.climb.ClimberMove;
import frc.robot.commands.climb.LeftClimberMove;
import frc.robot.commands.climb.SetPowerLeft;
import frc.robot.commands.climb.SetPowerRight;
import frc.robot.commands.climb.RightClimberMove;
import frc.robot.commands.climb.ZeroClimber;
import frc.robot.commands.climb.ZeroLeftClimber;
import frc.robot.commands.climb.ZeroRightClimber;
import frc.robot.commands.intake.SpinIntake;
import frc.robot.commands.pivot.ManualPivot;
import frc.robot.commands.pivot.PivotToPosition;
import frc.robot.commands.shooter.ShootCommand;
import frc.robot.commands.ResyncEncoders;
import frc.robot.commands.ResetHeading;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
 
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final PivotSubsystem pivotSubsystem = new PivotSubsystem();
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    private final ClimbSubsystem climbSubsystem = new ClimbSubsystem();

    private final Joystick driverJoystick = new Joystick(OIConstants.kDriverControllerPort);
    private final Joystick driverstationTop = new Joystick(OIConstants.kTopHalfButtonBoardPort);
    private final Joystick driverstationBottom = new Joystick(OIConstants.kBottomHalfButtonBoardPort);
    private final Joystick debug_secondary = new Joystick(4);

    Trigger X1 = new JoystickButton(driverJoystick, 1);
    Trigger O2 = new JoystickButton(driverJoystick, 2);
    Trigger Square3 = new JoystickButton(driverJoystick, 3);
    Trigger Triangle4 = new JoystickButton(driverJoystick, 4);
    Trigger leftShoulder5 = new JoystickButton(driverJoystick, 5);
    Trigger rightShoulder6 = new JoystickButton(driverJoystick, 6);
    Trigger leftTrigger7 = new JoystickButton(driverJoystick, 7);
    Trigger rightTrigger8 = new JoystickButton(driverJoystick, 8);
    Trigger leftStickPress9 = new JoystickButton(driverJoystick, 9);
    Trigger rightStickPress10 = new JoystickButton(driverJoystick, 10);
    
    Trigger driverPOVNorth = new POVButton(driverJoystick, 0);
    Trigger driverPOVSouth = new POVButton(driverJoystick, 180);
    Trigger driverPOVWest = new POVButton(driverJoystick, 270);
    Trigger driverPOVEast = new POVButton(driverJoystick, 90);

    Trigger buttonT1 = new JoystickButton(driverstationTop, 1);
    Trigger buttonT2 = new JoystickButton(driverstationTop, 2);
    Trigger buttonT3 = new JoystickButton(driverstationTop, 3);
    Trigger buttonT4 = new JoystickButton(driverstationTop, 4);
    Trigger buttonT5 = new JoystickButton(driverstationTop, 5);
    Trigger buttonT6 = new JoystickButton(driverstationTop, 6);
    Trigger buttonT7 = new JoystickButton(driverstationTop, 7);
    Trigger buttonT8 = new JoystickButton(driverstationTop, 8);
    Trigger buttonT9 = new JoystickButton(driverstationTop, 9);
    Trigger buttonT10 = new JoystickButton(driverstationTop, 10);

    Trigger buttonB1 = new JoystickButton(driverstationBottom, 1);
    Trigger buttonB2 = new JoystickButton(driverstationBottom, 2);
    Trigger buttonB3 = new JoystickButton(driverstationBottom, 3);
    Trigger buttonB4 = new JoystickButton(driverstationBottom, 4);
    Trigger buttonB5 = new JoystickButton(driverstationBottom, 5);
    Trigger buttonB6 = new JoystickButton(driverstationBottom, 6);
    Trigger buttonB7 = new JoystickButton(driverstationBottom, 7);
    Trigger buttonB8 = new JoystickButton(driverstationBottom, 8);
    Trigger buttonB9 = new JoystickButton(driverstationBottom, 9);
    Trigger buttonB10 = new JoystickButton(driverstationBottom, 10);

    Trigger buttonD7 = new JoystickButton(debug_secondary, 7);
    Trigger buttonD8 = new JoystickButton(debug_secondary, 8);
    Trigger buttonD9 = new JoystickButton(debug_secondary, 9);

    public RobotContainer() {
        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                swerveSubsystem,
                () -> driverJoystick.getRawAxis(OIConstants.kDriverYAxis),
                () -> -driverJoystick.getRawAxis(OIConstants.kDriverXAxis),
                () -> driverJoystick.getRawAxis(OIConstants.kDriverRotAxis),
                () -> !driverJoystick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));

        configureButtonBindings();
    }

    private void configureButtonBindings() {

        X1.whileTrue(new SpinIntake(intakeSubsystem, shooterSubsystem, -0.5));
        //O2.whileTrue(new ShootCommand(shooterSubsystem)); 
        //Square3.onTrue(new PivotToPositionPID(pivotSubsystem, 0));
        Triangle4.onTrue(new ResetHeading(swerveSubsystem));
        leftShoulder5.whileTrue(new ShootCommand(shooterSubsystem, 0.5));
        //rightShoulder6.whileTrue(new SpinIntake(intakeSubsystem, shooterSubsystem, -0.5));
        leftTrigger7.whileTrue(new ShootCommand(shooterSubsystem, 1));
        rightTrigger8.whileTrue(new SpinIntake(intakeSubsystem, shooterSubsystem, 0.5));
        leftStickPress9.onTrue(new ResyncEncoders(swerveSubsystem));
        // rightStickPress10.onTrue(new ResetHeading(swerveSubsystem));

        buttonT1.whileTrue(new LeftClimberMove(climbSubsystem, -0.5));
        buttonT2.whileTrue(new ClimberMove(climbSubsystem, -0.5)); 
        buttonT3.whileTrue(new RightClimberMove(climbSubsystem, -0.5));
        buttonT4.whileTrue(new SpinIntake(intakeSubsystem, shooterSubsystem, 0.5));
        buttonT5.onTrue(new ResyncEncoders(swerveSubsystem));
        buttonT6.whileTrue(new LeftClimberMove(climbSubsystem, 0.5));
        buttonT7.whileTrue(new ClimberMove(climbSubsystem, 0.5));
        buttonT8.whileTrue(new RightClimberMove(climbSubsystem, 0.5));
        buttonT9.whileTrue(new SpinIntake(intakeSubsystem, shooterSubsystem, -0.5));
        buttonT10.onTrue(new ResetHeading(swerveSubsystem));

        buttonB1.onTrue(new ZeroLeftClimber(climbSubsystem));
        buttonB2.onTrue(new ZeroClimber(climbSubsystem));
        buttonB3.onTrue(new ZeroRightClimber(climbSubsystem));
        // buttonB4.onTrue(new PivotToPositionPID(pivotSubsystem, 0));
        // buttonB5.whileTrue(new SpinIntake(intakeSubsystem, shooterSubsystem, 0.5));
        //buttonB6.onTrue(new PivotToPosition(pivotSubsystem, 0.289));
        // buttonB7.whileTrue(new RightClimberMove(climbSubsystem, 0.5));
        // buttonB8.whileTrue(new RightClimberMove(climbSubsystem, -0.5));
        //buttonB9.whileTrue(new SetPowerLeft(pivotSubsystem, 0.10));
        //buttonB10.whileTrue(new SetPowerRight(pivotSubsystem, 0.10));
        // buttonD7.whileTrue(new ManualPivot(pivotSubsystem,() -> debug_secondary.getRawAxis(OIConstants.kDriverYAxis)));
        buttonD8.whileTrue(new PivotToPosition(pivotSubsystem, 20));
        buttonD9.whileTrue(new PivotToPosition(pivotSubsystem, -20));
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
