// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.subsystems.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.*;
import frc.robot.LimelightHelpers.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.robot.Constants.*;

import frc.robot.generated.TunerConstants;

public class RobotContainer {

    // ===Variables===//
    private double MaxSpeed = 0.25 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    // ===Controllers===//
    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController aux = new CommandXboxController(1);
    private final Joystick OI = new Joystick(2);

    // ===Subsystems===//

    //Declare subsystems
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final Intake intake = new Intake();
    private final Shooter shooter = new Shooter();
    private final Turret turret = new Turret();
    private final Climber climber = new Climber();
    private final Indexer indexer = new Indexer();

    // ===Extra Systems===//

    // ===COMMANDS===//
    private final Command RunIntakeCommand;
    private final Command RunShooterCommand;
    private final Command RunTurretRightCommand;
    private final Command RunTurretLeftCommand;
    private final Command AutoTurretCommand;
    private final Command AutoShooterCommand;
    private final Command LiftIntakeCommand;
    private final Command ShootBallCommand;
    private final Command ManualLiftIntakeCommand;
    private final Command RunClimberCommand;
    private final Command ManualClimberCommand;
    private final Command DisableAutoTurretCommand;

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    //===Buttons===//
    private final JoystickButton ParkButton;
    private final JoystickButton DisableAutoTurretButton;
    private final JoystickButton ResetRobotButton;
    private final JoystickButton ZeroEncodersButton;
    private final JoystickButton DisableStateButton;
    private final JoystickButton ShootingModeButton;

    // ===Logging===//
    private final Telemetry logger = new Telemetry(MaxSpeed);


    public RobotContainer() {
                
        //Initialize Commands
        RunIntakeCommand = new RunIntake(intake, -0.75);
        RunShooterCommand = new RunShooter(shooter, 0);
        RunTurretRightCommand = new ManualTurret(turret, .06);
        RunTurretLeftCommand = new ManualTurret(turret, -.06);
        AutoTurretCommand = new AutoTurret(turret);
        AutoShooterCommand = new AutoShooter(shooter, 1);
        LiftIntakeCommand = new LiftIntake(intake);
        ShootBallCommand = new ShootBall(shooter, indexer);
        ManualLiftIntakeCommand = new ManualLiftIntake(intake, (-joystick.getLeftY()));
        RunClimberCommand = new RunClimber(null);
        ManualClimberCommand = new ManualClimber(climber, (-joystick.getRightY()));
        DisableAutoTurretCommand = new DisableAutoTurret();

        //Initialize Buttons
        ParkButton = new JoystickButton(OI, ControlConstants.LaunchPadSwitch3);
        DisableAutoTurretButton = new JoystickButton(OI, ControlConstants.LaunchPadSwitch8);
        ResetRobotButton = new JoystickButton(OI, ControlConstants.LaunchPadSwitch1top);
        ZeroEncodersButton = new JoystickButton(OI, ControlConstants.LaunchPadSwitch2top);
        DisableStateButton = new JoystickButton(OI, ControlConstants.LaunchPadSwitch4);
        ShootingModeButton = new JoystickButton(OI, ControlConstants.LaunchPadSwitch7);

        drivetrain.seedFieldCentric();

        //Set Default Commands For Subsystems
        turret.setDefaultCommand(AutoTurretCommand);
        intake.setDefaultCommand(ManualLiftIntakeCommand);
        climber.setDefaultCommand(ManualClimberCommand);

        configureBindings();
    }

    /** 
     * Method to configure control bindings
     */
    private void configureBindings() {

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // Park mode for the robot to disable movement and X the wheels
        ParkButton.whileTrue(drivetrain.applyRequest(() -> brake));
        DisableAutoTurretButton.whileTrue(DisableAutoTurretCommand);
        

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        //joystick.x().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        //joystick.y().whileTrue(RunIntakeCommand);

        drivetrain.registerTelemetry(logger::telemeterize);


        //Subsystem Buttons on Main Driver Controller
        joystick.a().whileTrue(RunIntakeCommand);
        joystick.b().whileTrue(ShootBallCommand);
        //joystick.x().onTrue(); //Add command to swap between field and robot oriented driving mode
        //joystick.y().onTrue(); // Add command to swap between fast and slow driving mode
        joystick.rightBumper().whileTrue(RunTurretRightCommand);
        joystick.leftBumper().whileTrue(RunTurretLeftCommand);

        //Subsystem Buttons on Aux Controller
        aux.x().onTrue(LiftIntakeCommand);
        aux.b().onTrue(RunClimberCommand);

    }
    
    /**
     * Method to determine Autonomous command
     * 
     * @return Autonomous Command
     */
    public Command getAutonomousCommand() {
        // Simple drive forward auton
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
            // Reset our field centric heading to match the robot
            // facing away from our alliance station wall (0 deg).
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            // Then slowly drive forward (away from us) for 5 seconds.
            drivetrain.applyRequest(() ->
                drive.withVelocityX(0.5)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            )
            .withTimeout(5.0),
            // Finally idle for the rest of auton
            drivetrain.applyRequest(() -> idle)
        );
    }

    public void UpdateStatus() {
        SmartDashboard.putNumber("TX", LimelightHelpers.getTX("limelight-turret"));
    }
}
