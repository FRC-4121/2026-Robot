// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.hardware.*;

import frc.robot.subsystems.*;
import frc.robot.commands.*;
import frc.robot.extras.Ballistics2026;
import frc.robot.Constants.*;
import frc.robot.generated.TunerConstants;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import com.pathplanner.lib.auto.*;

public class RobotContainer {

    //===Declare Controllers===//
    private final CommandXboxController joystick;
    private final CommandXboxController aux;
    private final Joystick OI;

    //===Declare Subsystems===//
    public final CommandSwerveDrivetrain drivetrain;
    private final Intake intake;
    private final Shooter shooter;
    private final Turret turret;
    private final Climber climber;
    private final Indexer indexer;

    private final Pigeon2 pigeon;

    private final Ballistics2026 myBallistics;

    //===Declare Commands===//
    private final Command RunIntakeCommand;
    private final Command RunTurretRightCommand;
    private final Command RunTurretLeftCommand;
    private final Command AutoTurretCommand;
    private final Command LiftIntakeCommand;
    private final Command ShootBallCommand;
    private final Command AutoShootCommand;
    private final Command ManualLiftIntakeCommand;
    private final Command RunClimberCommand;
    private final Command ManualClimberCommand;
    private final Command DisableAutoTurretCommand;
    private final Command EnableAutoTurretCommand;
    private final Command ChangeDrivingSpeedCommand;
    private final Command DisableStateFalseCommand;
    private final Command DisableStateTrueCommand;
    private final Command ZeroEncodersCommand;
    private final Command ShooterModeCommand;
    private final Command ShuttleModeCommand;
    private final Command AutoIntakeCommand;
    private final Command MixHopperCommand;

    //===Declare Buttons===//
    private final JoystickButton ParkButton;
    private final JoystickButton DisableAutoTurretButton;
    private final JoystickButton ResetRobotButton;
    private final JoystickButton ZeroEncodersButton;
    private final JoystickButton DisableStateButton;
    private final JoystickButton ShootingModeButton;

    //===Swerve Drive Variables===//
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * DriveConstants.slowModeMultiplier; // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond)  * DriveConstants.slowModeAngularMultiplier; // 3/4 of a rotation per second max angular velocity

    //===Swerve Drive Bindings===//
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    //===Declare Logging===//
    private final Telemetry logger = new Telemetry(MaxSpeed);

    //===Declare Extra Systems===//

    // ===PathPlanner=== //

    // Declare PathPlanner variables
     private final SendableChooser<Command> autoChooser;

    /**
     * Create a new RobotContainer
     */
    public RobotContainer() {
    
        // Initialize Subsystems
        drivetrain = TunerConstants.createDrivetrain();
        intake = new Intake();
        shooter = new Shooter();
        turret = new Turret();
        climber = new Climber();
        indexer = new Indexer();

        pigeon = new Pigeon2(13);

        // Initialize controllers
        joystick = new CommandXboxController(0);
        aux = new CommandXboxController(1);
        OI = new Joystick(2);

        //Creates Ballistics Used by Shooter
        myBallistics = new Ballistics2026(
            MechanismConstants.kTurretCameraHeight, 
            MechanismConstants.kTargetHeight, 
            MechanismConstants.kShooterWheelDiameter,
            MechanismConstants.kShooterDriveRatio);

        // Initialize Buttons
        ParkButton = new JoystickButton(OI, ControlConstants.LaunchPadSwitch3);
        DisableAutoTurretButton = new JoystickButton(OI, ControlConstants.LaunchPadSwitch8);
        ResetRobotButton = new JoystickButton(OI, ControlConstants.LaunchPadSwitch1top);
        ZeroEncodersButton = new JoystickButton(OI, ControlConstants.LaunchPadSwitch2top);
        DisableStateButton = new JoystickButton(OI, ControlConstants.LaunchPadSwitch4);
        ShootingModeButton = new JoystickButton(OI, ControlConstants.LaunchPadSwitch7);

        //Initialize Commands
        RunIntakeCommand = new RunIntake(intake, -0.5);
        RunTurretRightCommand = new ManualTurret(turret, -.1);
        RunTurretLeftCommand = new ManualTurret(turret, .1);
        AutoTurretCommand = new AutoTurret(turret);
        LiftIntakeCommand = new LiftIntake(intake);
        ShootBallCommand = new ShootBall(shooter, indexer, intake, myBallistics);
        AutoShootCommand = new AutoShoot(shooter, indexer);
        ManualLiftIntakeCommand = new ManualLiftIntake(intake, aux);
        RunClimberCommand = new RunClimber(climber);
        ManualClimberCommand = new ManualClimber(climber, aux);
        DisableAutoTurretCommand = new DisableAutoTurret(false);
        EnableAutoTurretCommand = new DisableAutoTurret(true);
        ChangeDrivingSpeedCommand = new ChangeDrivingSpeed();
        DisableStateTrueCommand = new DisableState(true);
        DisableStateFalseCommand = new DisableState(false);
        ZeroEncodersCommand = new ZeroEncoders(intake, turret, climber);
        ShooterModeCommand = new ChangeShootingMode(true);
        ShuttleModeCommand = new ChangeShootingMode(false);
        AutoIntakeCommand = new AutoIntake(intake, -.75);
        MixHopperCommand = new MixHopper(indexer);

        // Set Default Commands For Subsystems
        turret.setDefaultCommand(AutoTurretCommand);
        intake.setDefaultCommand(ManualLiftIntakeCommand);
        climber.setDefaultCommand(ManualClimberCommand);
        indexer.setDefaultCommand(MixHopperCommand);

        // Register named commands for PathPlanner
        NamedCommands.registerCommand("Intake", AutoIntakeCommand);
        NamedCommands.registerCommand("Shoot", ShootBallCommand);
        NamedCommands.registerCommand("Lift Intake", LiftIntakeCommand);
        NamedCommands.registerCommand("Climb", RunClimberCommand);
        
        // Set field centric drive
        drivetrain.seedFieldCentric();

        // Bind commands to buttons
        configureBindings();

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        turret.getHubInfo();
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
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed * DriveConstants.slowModeMultiplier) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed * DriveConstants.slowModeMultiplier) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate * DriveConstants.slowModeAngularMultiplier) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );
        
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        joystick.x().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);

        //Subsystem Buttons on Main Driver Controller
        joystick.a().whileTrue(RunIntakeCommand);
        joystick.b().whileTrue(ShootBallCommand);
        //joystick.x().onTrue(); //Add command to swap between field and robot oriented driving mode
        joystick.y().onTrue(ChangeDrivingSpeedCommand);
        joystick.rightBumper().whileTrue(RunTurretRightCommand);
        joystick.leftBumper().whileTrue(RunTurretLeftCommand);

        //Subsystem Buttons on Aux Controller
        aux.x().onTrue(LiftIntakeCommand);
        aux.b().onTrue(RunClimberCommand);

        //OI Buttons
        DisableStateButton.onTrue(DisableStateTrueCommand);
        DisableStateButton.onFalse(DisableStateFalseCommand);
        ZeroEncodersButton.onTrue(ZeroEncodersCommand);
        ParkButton.whileTrue(drivetrain.applyRequest(() -> brake));
        DisableAutoTurretButton.onTrue(DisableAutoTurretCommand);
        DisableAutoTurretButton.onFalse(EnableAutoTurretCommand);
        ShootingModeButton.onTrue(ShooterModeCommand);
        ShootingModeButton.onFalse(ShuttleModeCommand);

    }
    
    /**
     * Method to determine Autonomous command
     * 
     * @return Autonomous Command
     */
    public Command getAutonomousCommand() {
        
        return autoChooser.getSelected();

        // Simple drive forward auton
        // final var idle = new SwerveRequest.Idle();
        // return Commands.sequence(
        //     // Reset our field centric heading to match the robot
        //     // facing away from our alliance station wall (0 deg).
        //     drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
        //     // Then slowly drive forward (away from us) for 5 seconds.
        //     drivetrain.applyRequest(() ->
        //         drive.withVelocityX(0)
        //             .withVelocityY(0)
        //             .withRotationalRate(0)
        //     )
        //     .withTimeout(5.0),
        //     // Finally idle for the rest of auton
        //     drivetrain.applyRequest(() -> idle)
        // );
    }

    /**
     * Update target yaw from limelight camera
     */
    public void UpdateStatus() {
        //SmartDashboard.putNumber("TX", LimelightHelpers.getTX("limelight-turret"));
        //SmartDashboard.putNumber("Inake Angle", intake.getPosition());
        //SmartDashboard.putNumber("Climber Pos", climber.getPosition());
        //SmartDashboard.putNumber("Turret Angle", turret.getPosition());
        //turret.getHubInfo();
        SmartDashboard.putNumber("Gyro Data", pigeon.getYaw().getValueAsDouble());
        SmartDashboard.putNumber("Target Speed", MechanismConstants.targetVelocity);
        SmartDashboard.putNumber("Shooter Speed", shooter.getWheelVelocity());
    }

    /**
     * Getting alliance color from driver's station
     */
    public void getAlliance(){
        Optional<Alliance> allianceColor = DriverStation.getAlliance();
        if (allianceColor.isPresent()) {
            if (allianceColor.get() == Alliance.Red) {
                Mutables.blueAlliance = false;
            }
            else if (allianceColor.get() == Alliance.Blue) {
                Mutables.blueAlliance = true;
            }
        }
        else {
           Mutables.blueAlliance = true;
        }
    }

    /**
     * Command to set field centric drive
     * 
     * @return Field centric drive command
     */
    public Command setFieldCentricGyro() {

        return drivetrain.runOnce(drivetrain::seedFieldCentric);

    }
}
