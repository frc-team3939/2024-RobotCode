package frc.robot;

import org.photonvision.PhotonCamera;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.Constants.OIConstants;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.VisionandIntake;
import frc.robot.commands.climb.ClimberMove;
import frc.robot.commands.climb.LeftClimberMove;
import frc.robot.commands.climb.RightClimberMove;
import frc.robot.commands.climb.ZeroClimber;
import frc.robot.commands.climb.ZeroLeftClimber;
import frc.robot.commands.climb.ZeroRightClimber;
import frc.robot.commands.intake.Extake;
import frc.robot.commands.intake.SpinIntake;
import frc.robot.commands.shooter.ShootAmp;
import frc.robot.commands.shooter.ShootCommand;
import frc.robot.commands.shooter.ShooterBreak;
import frc.robot.commands.shooter.ShooterCoast;
import frc.robot.commands.ResyncEncoders;
import frc.robot.commands.AmpVision;
import frc.robot.commands.RedoOffsets;
import frc.robot.commands.ResetHeading;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
 
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    private final ClimbSubsystem climbSubsystem = new ClimbSubsystem();

    private final Joystick driverJoystick = new Joystick(OIConstants.kDriverControllerPort);
    private final Joystick driverstationTop = new Joystick(OIConstants.kTopHalfButtonBoardPort);
    private final Joystick driverstationBottom = new Joystick(OIConstants.kBottomHalfButtonBoardPort);
    // private final Joystick debug_secondary = new Joystick(4);

    private final SendableChooser<Command> autoChooser;

    private final PhotonCamera RedvisionCamera = new PhotonCamera("Life_Cam_Red");
    private final PhotonCamera BluevisionCamera = new PhotonCamera("Life_Cam_Blue");


    Trigger X1 = new JoystickButton(driverJoystick, 1);
    Trigger O2 = new JoystickButton(driverJoystick, 2);
    Trigger Square3 = new JoystickButton(driverJoystick, 3);
    Trigger Triangle4 = new JoystickButton(driverJoystick, 4);
    Trigger leftShoulder5 = new JoystickButton(driverJoystick, 5);
    Trigger rightShoulder6 = new JoystickButton(driverJoystick, 6);
    Trigger leftTrigger7 = new JoystickButton(driverJoystick, 7);
    Trigger rightTrigger8 = new JoystickButton(driverJoystick, 8);
    // Trigger leftTrigger7 = new JoystickButton(driverJoystick, 7);
    // Trigger rightTrigger8 = new JoystickButton(driverJoystick, 8);
    Trigger leftStickPress9 = new JoystickButton(driverJoystick, 9);
    Trigger rightStickPress10 = new JoystickButton(driverJoystick, 10);
    
    Trigger dPadNorth = new POVButton(driverJoystick, 0);
    Trigger dPadSouth = new POVButton(driverJoystick, 180);
    Trigger dPadWest = new POVButton(driverJoystick, 270);
    Trigger dPadEast = new POVButton(driverJoystick, 90);

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

    // Trigger buttonD7 = new JoystickButton(debug_secondary, 7);
    // Trigger buttonD8 = new JoystickButton(debug_secondary, 8);
    // Trigger buttonD9 = new JoystickButton(debug_secondary, 9);

    public RobotContainer() {
        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                swerveSubsystem,
                () -> -driverJoystick.getRawAxis(OIConstants.kDriverYAxis),
                () -> -driverJoystick.getRawAxis(OIConstants.kDriverXAxis),
                () -> -driverJoystick.getRawAxis(OIConstants.kDriverRotAxis),
                () -> !driverJoystick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));

        configureButtonBindings();
        
        NamedCommands.registerCommand("SpinIntake", new SpinIntake(intakeSubsystem, shooterSubsystem, 0.5));
        NamedCommands.registerCommand("SpinIntake2", new SpinIntake(intakeSubsystem, shooterSubsystem, 0.5));
        NamedCommands.registerCommand("ExtakeAuto", new Extake(intakeSubsystem, shooterSubsystem, -0.5));
        NamedCommands.registerCommand("ShootCommand", new ShootCommand(shooterSubsystem, 1));
        NamedCommands.registerCommand("ResetHeading", new ResetHeading(swerveSubsystem));
        NamedCommands.registerCommand("ResyncEncoders", new ResyncEncoders(swerveSubsystem));
        NamedCommands.registerCommand("ShooterCoast", new ShooterCoast(shooterSubsystem));
        NamedCommands.registerCommand("ShooterBrake", new ShooterBreak(shooterSubsystem));
        NamedCommands.registerCommand("Vision", new VisionandIntake(swerveSubsystem, intakeSubsystem, shooterSubsystem, () -> BluevisionCamera.getLatestResult(), true));

        // Build an auto chooser. This will use Commands.none() as the default option.
        autoChooser = AutoBuilder.buildAutoChooser();

        // Another option that allows you to specify the default auto by its name
        // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

        SmartDashboard.putData("Auto Chooser", autoChooser);
  }
  

    private void configureButtonBindings() {

        X1.whileTrue(new Extake(intakeSubsystem, shooterSubsystem, -0.5));
        //O2.whileTrue(new ShootCommand(shooterSubsystem)); 
        //Square3.onTrue(new PivotToPositionPID(pivotSubsystem, 0));
        Triangle4.onTrue(new ResetHeading(swerveSubsystem));
        // leftShoulder5.whileTrue(new VisionandIntake(swerveSubsystem, intakeSubsystem, shooterSubsystem, () -> BluevisionCamera.getLatestResult(), true));
        rightShoulder6.whileTrue(new ShootAmp(shooterSubsystem, 0.27));
        leftTrigger7.whileTrue(new ShootCommand(shooterSubsystem, 1));
        rightTrigger8.whileTrue(new SpinIntake(intakeSubsystem, shooterSubsystem, 0.80));
        leftStickPress9.onTrue(new ResyncEncoders(swerveSubsystem));
        // rightStickPress10.onTrue(new ResetHeading(swerveSubsystem));
        dPadNorth.whileTrue(new AmpVision(swerveSubsystem, () -> RedvisionCamera.getLatestResult(), true));

        buttonT1.whileTrue(new LeftClimberMove(climbSubsystem, 1));
        buttonT2.whileTrue(new ClimberMove(climbSubsystem, 1)); 
        buttonT3.whileTrue(new RightClimberMove(climbSubsystem, 1));
        buttonT4.onTrue(new ResyncEncoders(swerveSubsystem));
        buttonT5.onTrue(new ResetHeading(swerveSubsystem));
        buttonT6.whileTrue(new ZeroLeftClimber(climbSubsystem));
        buttonT7.whileTrue(new ZeroClimber(climbSubsystem));
        buttonT8.whileTrue(new ZeroRightClimber(climbSubsystem));
        //buttonT9.whileTrue(new Extake(intakeSubsystem, shooterSubsystem, -0.5));
        //buttonT10.onTrue(new ResetHeading(swerveSubsystem));

        buttonB1.whileTrue(new LeftClimberMove(climbSubsystem, -1));
        buttonB2.whileTrue(new ClimberMove(climbSubsystem, -1));
        buttonB3.whileTrue(new RightClimberMove(climbSubsystem, -1));
        // buttonB4.whileTrue(new SpinIntake(intakeSubsystem, shooterSubsystem, 0.5));
        buttonB5.whileTrue(new ShootCommand(shooterSubsystem, 0.27));
        buttonB6.onTrue(new RedoOffsets(swerveSubsystem));
        // buttonB6.onTrue(new));
        // buttonB7.onTrue(new));
        // buttonB8.onTrue(new));
        // buttonB8.whileTrue(new RightClimberMove(climbSubsystem, -0.5));
        buttonB9.whileTrue(new SpinIntake(intakeSubsystem, shooterSubsystem, 0.5));
        buttonB10.whileTrue(new Extake(intakeSubsystem, shooterSubsystem, -0.5));

        // buttonD7.whileTrue(new ManualPivot(pivotSubsystem,() -> debug_secondary.getRawAxis(OIConstants.kDriverYAxis)));
        // buttonD8.whileTrue(new PivotToPosition(pivotSubsystem, 20));
        // buttonD9.whileTrue(new PivotToPosition(pivotSubsystem, -20));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
  }
    
}
