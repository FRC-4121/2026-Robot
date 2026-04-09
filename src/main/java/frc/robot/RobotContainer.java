// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
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
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import com.pathplanner.lib.auto.*;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import frc.robot.LumaHelpers;
import java.util.Optional;

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
    private final Indexer indexer;

    private final Pigeon2 pigeon;

    private final Ballistics2026 myBallistics;

    //===Declare Commands===//
    private final Command RunIntakeCommand;
    private final Command RunTurretRightCommand;
    private final Command RunTurretLeftCommand;
    private final Command LiftIntakeCommand;
    private final Command ShootBallCommand;
    private final Command AutoShootCommand;
    private final Command ManualLiftIntakeCommand;
    private final Command DisableAutoTurretCommand;
    private final Command EnableAutoTurretCommand;
    private final Command ChangeDrivingSpeedCommand;
    private final Command DisableStateFalseCommand;
    private final Command DisableStateTrueCommand;
    private final Command ZeroEncodersCommand;
    private final Command ShooterModeCommand;
    private final Command ShuttleModeCommand;
    private final Command AutoIntakeCommand;
    private final Command StopAutoShootCommand;

    //===Declare Buttons===//
    private final JoystickButton ParkButton;
    private final JoystickButton DisableAutoTurretButton;
    private final JoystickButton ResetRobotButton;
    private final JoystickButton ZeroEncodersButton;
    private final JoystickButton DisableStateButton;
    private final JoystickButton ShootingModeButton;
    private final JoystickButton LiftIntakeButton;

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

    //===Declare Camera Variables===//
    public static PhotonCamera frontCamera;
    public static PhotonCamera backCamera;
    public static PhotonCamera leftCamera;
    public static Transform3d kRobotToFrontCam;
    public static Transform3d kRobotToBackCam;
    public static Transform3d kRobotToLeftCam;

    //===Declare Field Pose Variables===//
    Field2d frontCamPose = new Field2d();
    Field2d backCamPose = new Field2d();
    Field2d leftCamPose = new Field2d();

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
        indexer = new Indexer();

        pigeon = new Pigeon2(13);

        // Initialize controllers
        joystick = new CommandXboxController(0);
        aux = new CommandXboxController(1);
        OI = new Joystick(2);

        //Creates Ballistics Used by Shooter
        myBallistics = new Ballistics2026(
            MechanismConstants.kShooterHeight, 
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
        LiftIntakeButton = new JoystickButton(OI, ControlConstants.LaunchPadButton4);

        //Initialize Commands
        RunIntakeCommand = new RunIntake(intake, -0.75);
        RunTurretRightCommand = new ManualTurret(turret, -.1);
        RunTurretLeftCommand = new ManualTurret(turret, .1);
        LiftIntakeCommand = new LiftIntake(intake);
        ShootBallCommand = new ShootBall(shooter, indexer, intake, myBallistics);
        AutoShootCommand = new AutoShoot(shooter, indexer, intake, myBallistics);
        ManualLiftIntakeCommand = new ManualLiftIntake(intake, aux);
        DisableAutoTurretCommand = new DisableAutoTurret(false);
        EnableAutoTurretCommand = new DisableAutoTurret(true);
        ChangeDrivingSpeedCommand = new ChangeDrivingSpeed();
        DisableStateTrueCommand = new DisableState(true);
        DisableStateFalseCommand = new DisableState(false);
        ZeroEncodersCommand = new ZeroEncoders(intake, turret);
        ShooterModeCommand = new ChangeShootingMode(true);
        ShuttleModeCommand = new ChangeShootingMode(false);
        AutoIntakeCommand = new AutoIntake(intake, -.75);
        StopAutoShootCommand = new StopAutoShoot();

        // Set Default Commands For Subsystems
        intake.setDefaultCommand(ManualLiftIntakeCommand);
    

        // Register named commands for PathPlanner
        NamedCommands.registerCommand("Intake", AutoIntakeCommand);
        NamedCommands.registerCommand("Shoot", AutoShootCommand);
        NamedCommands.registerCommand("Stop Shoot", StopAutoShootCommand);
        NamedCommands.registerCommand("Lift Intake", LiftIntakeCommand);
        
        
        // Set field centric drive
        drivetrain.seedFieldCentric();

        // Bind commands to buttons
        configureBindings();

        // Create vision cameras
        createCameras();

        // Create autonomous command chooser and add to dashboard
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);


    }

    /**
     * Create Luma vision cameras for pose estimation
     */
    public void createCameras() {

        frontCamera = new PhotonCamera("frontcam");
        backCamera = new PhotonCamera("backcam");
        leftCamera = new PhotonCamera("leftcam");

        kRobotToFrontCam = new Transform3d(new Translation3d(0.554, -0.234, 0.384),
             new Rotation3d(0, 0, 0));
        kRobotToBackCam = new Transform3d(new Translation3d(-0.283, 0.0, 0.269),
             new Rotation3d(0, Math.PI/12, Math.PI));
        kRobotToLeftCam = new Transform3d(new Translation3d(0.470, 0.337, 0.384),
             new Rotation3d(0, 0, 0.5*Math.PI));

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

        //OI Buttons
        DisableStateButton.onTrue(DisableStateTrueCommand);
        DisableStateButton.onFalse(DisableStateFalseCommand);
        ZeroEncodersButton.onTrue(ZeroEncodersCommand);
        ParkButton.whileTrue(drivetrain.applyRequest(() -> brake));
        DisableAutoTurretButton.onTrue(DisableAutoTurretCommand);
        DisableAutoTurretButton.onFalse(EnableAutoTurretCommand);
        ShootingModeButton.onTrue(ShooterModeCommand);
        ShootingModeButton.onFalse(ShuttleModeCommand);
        LiftIntakeButton.onTrue(LiftIntakeCommand);

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
        SmartDashboard.putBoolean("Stop Auto Shoot", MechanismConstants.stopAutoShooter);
        SmartDashboard.putBoolean("Slow Mode?", Mutables.isSlowMode);
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

    /**
     * Updates robots position on the field from cameras
     */
    public void updateRobotPose(){

        // Create local variables
        double frontPoseX = 0;
        double frontPoseY = 0;
        double frontDist = 0;
        double leftPoseX = 0;
        double leftPoseY = 0;
        double leftDist = 0;
        double backPoseX = 0;
        double backPoseY = 0;
        double backDist = 0;
        double avgDist = 0;
        double camCount = 0;
        double hubX = 0;
        double hubY = 0;

        if (Mutables.blueAlliance) {
            hubX = GeneralConstants.kBlueHub[0];
            hubY = GeneralConstants.kBlueHub[1];
        } else {
            hubX = GeneralConstants.kRedHub[0];
            hubY = GeneralConstants.kRedHub[1];
        }

        //Call pose estimation method
        Optional<EstimatedRobotPose> frontPose = LumaHelpers.getPose(frontCamera, kRobotToFrontCam);
        Optional<EstimatedRobotPose> backPose = LumaHelpers.getPose(backCamera, kRobotToBackCam);
        Optional<EstimatedRobotPose> leftPose = LumaHelpers.getPose(leftCamera, kRobotToLeftCam);
        SmartDashboard.putBoolean("Pose Found", frontPose.isPresent());
        if (frontPose.isPresent()) {
            EstimatedRobotPose est = frontPose.get();
            Pose2d frontPose2d = est.estimatedPose.toPose2d();
            frontPoseX = frontPose2d.getX();
            frontPoseY = frontPose2d.getY();
            frontDist = Math.sqrt( ((hubX - frontPoseX)*(hubX - frontPoseX)) + ((hubY - frontPoseY) * (hubY - frontPoseY)));
            camCount++;
            SmartDashboard.putNumber("front X Pose", frontPoseX);
            frontCamPose.setRobotPose(frontPose2d);
            SmartDashboard.putData("frontRobotPose", frontCamPose);
        }
        if (backPose.isPresent()) {
            EstimatedRobotPose est = backPose.get();
            Pose2d backPose2d = est.estimatedPose.toPose2d();
            backPoseX = backPose2d.getX();
            backPoseY = backPose2d.getY();
            backDist = Math.sqrt( ((hubX - backPoseX)*(hubX - backPoseX)) + ((hubY - backPoseY) * (hubY - backPoseY)));
            camCount++;
            SmartDashboard.putNumber("back X Pose", backPoseX);
            backCamPose.setRobotPose(backPose2d);
            SmartDashboard.putData("backRobotPose", backCamPose);
        }
        if (leftPose.isPresent()) {
            EstimatedRobotPose est = leftPose.get();
            Pose2d leftPose2d = est.estimatedPose.toPose2d();
            leftPoseX = leftPose2d.getX();
            leftPoseY = leftPose2d.getY();
            leftDist = Math.sqrt( ((hubX - leftPoseX)*(hubX - leftPoseX)) + ((hubY - leftPoseY) * (hubY - leftPoseY)));
            camCount++;
            SmartDashboard.putNumber("left X Pose", leftPoseX);
            leftCamPose.setRobotPose(leftPose2d);
            SmartDashboard.putData("leftRobotPose", leftCamPose);
        }   

        // Average Distance Calculation
        avgDist = (frontDist + backDist + leftDist) / camCount;
        MechanismConstants.targetDistance = avgDist;
        SmartDashboard.putNumber("Average Hub Distance", avgDist);
        SmartDashboard.putNumber("Target Distance", MechanismConstants.targetDistance);
    }
}
