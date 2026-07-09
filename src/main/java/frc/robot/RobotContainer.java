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
import frc.robot.extras.LumaCam;

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

    private final Ballistics2026 myBallistics;

    //===Declare Commands===//
    private final Command RunIntakeCommand;
    private final Command LiftIntakeCommand;
    //private final Command ShootBallCommand;
    private final Command AutoShootCommand;
    private final Command ManualLiftIntakeCommand;
    private final Command DisableAutoRotateCommand;
    private final Command EnableAutoRotateCommand;
    private final Command ChangeDrivingSpeedCommand;
    private final Command DisableStateFalseCommand;
    private final Command DisableStateTrueCommand;
    private final Command ZeroEncodersCommand;
    private final Command ShooterModeCommand;
    private final Command ShuttleModeCommand;
    private final Command AutoIntakeCommand;
    private final Command StopAutoShootCommand;
    private final Command StopAutoIntakeCommand;
    private final Command AutoShuttleCommand;
    private final Command IndexerOverrideOnCommand;
    private final Command IndexerOverrideOffCommand;
    private final Command SpoolShooterCommand;
    private final Command AutoSpoolShooterCommand;

    //===Declare Buttons===//
    private final JoystickButton ParkButton;
    private final JoystickButton DisableAutoRotateButton;
    private final JoystickButton ResetRobotButton;
    private final JoystickButton ZeroEncodersButton;
    private final JoystickButton DisableStateButton;
    private final JoystickButton ShootingModeButton;
    private final JoystickButton LiftIntakeButton;
    private final JoystickButton IndexerOverrideButton;
    private final JoystickButton ShooterSpoolButton;

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
    public static LumaCam frontCamera;
    public static LumaCam backCamera;
    public static LumaCam leftCamera;

    //===Declare Field Pose Variables===//
    Field2d frontCamPose = new Field2d();
    Field2d backCamPose = new Field2d();
    Field2d leftCamPose = new Field2d();
    Field2d robotPose = new Field2d();

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
        DisableAutoRotateButton = new JoystickButton(OI, ControlConstants.LaunchPadSwitch8);
        ResetRobotButton = new JoystickButton(OI, ControlConstants.LaunchPadSwitch1top);
        ZeroEncodersButton = new JoystickButton(OI, ControlConstants.LaunchPadSwitch2top);
        DisableStateButton = new JoystickButton(OI, ControlConstants.LaunchPadSwitch4);
        ShootingModeButton = new JoystickButton(OI, ControlConstants.LaunchPadSwitch7);
        LiftIntakeButton = new JoystickButton(OI, ControlConstants.LaunchPadButton3);
        IndexerOverrideButton = new JoystickButton(OI, ControlConstants.LaunchPadButton1);
        ShooterSpoolButton = new JoystickButton(OI, ControlConstants.LaunchPadSwitch6bottom);

        //Initialize Commands
        RunIntakeCommand = new RunIntake(intake, MechanismConstants.kIntakeSpeed);
        LiftIntakeCommand = new LiftIntake(intake);
        //ShootBallCommand = new ShootBall(shooter, indexer, intake, drivetrain, pigeon, myBallistics);
        AutoShootCommand = new AutoShoot(shooter, indexer, intake, drivetrain, myBallistics);
        ManualLiftIntakeCommand = new ManualLiftIntake(intake, aux);
        DisableAutoRotateCommand = new DisableAutoRotate(false);
        EnableAutoRotateCommand = new DisableAutoRotate(true);
        ChangeDrivingSpeedCommand = new ChangeDrivingSpeed();
        DisableStateTrueCommand = new DisableState(true);
        DisableStateFalseCommand = new DisableState(false);
        ZeroEncodersCommand = new ZeroEncoders(intake, turret);
        ShooterModeCommand = new ChangeShootingMode(true);
        ShuttleModeCommand = new ChangeShootingMode(false);
        AutoIntakeCommand = new AutoIntake(intake, MechanismConstants.kIntakeSpeed);
        StopAutoShootCommand = new StopAutoShoot();
        StopAutoIntakeCommand = new StopAutoIntake();
        AutoShuttleCommand = new AutoShuttle(shooter, indexer);
        IndexerOverrideOnCommand = new IndexerOverride(true);
        IndexerOverrideOffCommand = new IndexerOverride(false);
        SpoolShooterCommand = new SpoolShooter(shooter);
        AutoSpoolShooterCommand = new AutoSpoolShooter(shooter);

        // Set Default Commands For Subsystems
        intake.setDefaultCommand(ManualLiftIntakeCommand);
        shooter.setDefaultCommand(SpoolShooterCommand);
    

        // Register named commands for PathPlanner
        NamedCommands.registerCommand("Intake", AutoIntakeCommand);
        NamedCommands.registerCommand("Stop Intake", StopAutoIntakeCommand);
        NamedCommands.registerCommand("Shoot", AutoShootCommand);
        NamedCommands.registerCommand("Stop Shoot", StopAutoShootCommand);
        NamedCommands.registerCommand("Lift Intake", LiftIntakeCommand);
        NamedCommands.registerCommand("Shuttle", AutoShuttleCommand);
        NamedCommands.registerCommand("Spool Shooter", AutoSpoolShooterCommand);
        
        
        // Set field centric drive
        drivetrain.seedFieldCentric();

        // Bind commands to buttons
        configureBindings();

        // Create vision cameras
        createCameras();

        // Checks origional button status
        getButtonState();

        // Create autonomous command chooser and add to dashboard
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

    }

    /**
     * Create Luma vision cameras for pose estimation
     */
    public void createCameras() {

        //frontCamera = new LumaCam("frontcam", 0.568, 0.013, 0.248, 0, Math.PI/12, 0, drivetrain);
        backCamera = new LumaCam("backcam", -0.283, 0.0, 0.269, 0, Math.PI/12, Math.PI, drivetrain);
        leftCamera = new LumaCam("leftcam", 0.470, 0.337, 0.384, 0, 0, 0.5*Math.PI, drivetrain);

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

        // Reset the field-centric heading on x button press.
        joystick.x().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);

        //Subsystem Buttons on Main Driver Controller
        joystick.a().whileTrue(RunIntakeCommand);
        //joystick.b().whileTrue(ShootBallCommand);
        joystick.y().onTrue(ChangeDrivingSpeedCommand);

        //Subsystem Buttons on Aux Controller
        //aux.x().onTrue(LiftIntakeCommand);

        //OI Buttons
        DisableStateButton.onTrue(DisableStateTrueCommand);
        DisableStateButton.onFalse(DisableStateFalseCommand);
        ZeroEncodersButton.onTrue(ZeroEncodersCommand);
        ParkButton.whileTrue(drivetrain.applyRequest(() -> brake));
        DisableAutoRotateButton.onTrue(DisableAutoRotateCommand);
        DisableAutoRotateButton.onFalse(EnableAutoRotateCommand);
        ShootingModeButton.onTrue(ShooterModeCommand);
        ShootingModeButton.onFalse(ShuttleModeCommand);
        LiftIntakeButton.onTrue(LiftIntakeCommand);
        IndexerOverrideButton.onTrue(IndexerOverrideOnCommand);
        IndexerOverrideButton.onFalse(IndexerOverrideOffCommand);
        MechanismConstants.isShooterSpooling = ShooterSpoolButton.getAsBoolean();

    }

    public void getButtonState() {

        MechanismConstants.isDisableState = !DisableStateButton.getAsBoolean();
        //MechanismConstants.isShooterMode = ShootingModeButton.getAsBoolean();
        MechanismConstants.isRotateEnabled = !DisableAutoRotateButton.getAsBoolean();
        MechanismConstants.isIndexerOverride = IndexerOverrideButton.getAsBoolean();

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

        updateDashboard();
        //frontCamera.updatePose();
        backCamera.updatePose();
        leftCamera.updatePose();

    }

    /**
     * Update smart dashboard values
     */
    public void updateDashboard(){

        MechanismConstants.isShooterSpooling = ShooterSpoolButton.getAsBoolean();
        MechanismConstants.currentGyro = drivetrain.getCurrentGyro();
        SmartDashboard.putNumber("Gyro Data", MechanismConstants.currentGyro);
        SmartDashboard.putNumber("Target Speed", MechanismConstants.targetVelocity);
        SmartDashboard.putNumber("Shooter Speed", shooter.getWheelVelocity());
        SmartDashboard.putBoolean("Stop Auto Shoot", MechanismConstants.stopAutoShooter);
        SmartDashboard.putBoolean("Slow Mode?", Mutables.isSlowMode);
        SmartDashboard.putNumber("Intake Position", intake.getPosition());
        SmartDashboard.putNumber("Shooter Current", shooter.getShooterCurrent());
        SmartDashboard.putBoolean("Can Shoot?", MechanismConstants.canShoot);
        robotPose.setRobotPose(drivetrain.getCurrentPose());
        SmartDashboard.putData("Robot Pose", robotPose);
        SmartDashboard.putNumber("Target Angle", MechanismConstants.targetGyroAngle);
        SmartDashboard.putBoolean("Blue Alliance", Mutables.blueAlliance);
        SmartDashboard.putNumber("left tags", Mutables.leftTags);
        SmartDashboard.putNumber("front tags", Mutables.frontTags);
        SmartDashboard.putNumber("back tags", Mutables.backTags);
        SmartDashboard.putNumber("hub distance", MechanismConstants.hubDistance);
        SmartDashboard.putBoolean("Lined Up?", MechanismConstants.linedUp);
        SmartDashboard.putBoolean("Yaw Lined Up?", MechanismConstants.yawLinedUp);
        SmartDashboard.putNumber("Target Yaw", MechanismConstants.targetYaw);
        SmartDashboard.putBoolean("Yaw 1 to 5", MechanismConstants.yawLinedUp1);
        SmartDashboard.putBoolean("Yaw 5 to 10", MechanismConstants.yawLinedUp2);
        SmartDashboard.putBoolean("Yaw 10 to 20", MechanismConstants.yawLinedUp3);
        SmartDashboard.putBoolean("Yaw -1 to -5", MechanismConstants.yawLinedUp4);
        SmartDashboard.putBoolean("Yaw -5 to -10", MechanismConstants.yawLinedUp5);
        SmartDashboard.putBoolean("Yaw -10 to -20", MechanismConstants.yawLinedUp6);
        SmartDashboard.putBoolean("Back Tags Found?", MechanismConstants.backTags);
        SmartDashboard.putBoolean("Auto Rotate", MechanismConstants.isRotateEnabled);
        SmartDashboard.putBoolean("Shooter Mode?", MechanismConstants.isShooterMode);
        SmartDashboard.putBoolean("Shooter Spooling?", MechanismConstants.isShooterSpooling);
        SmartDashboard.putNumber("Velocity Output", MechanismConstants.velocityOutput);
        SmartDashboard.putNumber("Intake Lift Current", intake.getLiftCurrent());


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
    // public void updateRobotPose(){

    //     // Create local variables
    //     Pose2d frontPose2d = new Pose2d(-1, -1, Rotation2d.kZero);
    //     Pose2d leftPose2d = new Pose2d(-1, -1, Rotation2d.kZero);
    //     Pose2d backPose2d = new Pose2d(-1, -1, Rotation2d.kZero);
    //     double frontTime = 0;
    //     double backTime = 0;
    //     double leftTime = 0;
    //     double hubX = 0;
    //     double hubY = 0;

    //     if (Mutables.blueAlliance) {
    //         hubX = GeneralConstants.kBlueHub[0];
    //         hubY = GeneralConstants.kBlueHub[1];
    //     } else {
    //         hubX = GeneralConstants.kRedHub[0];
    //         hubY = GeneralConstants.kRedHub[1];
    //     }

    //     //Call pose estimation method
    //     Optional<EstimatedRobotPose> frontPose = LumaHelpers.getPose(frontCamera, kRobotToFrontCam, "front");
    //     Optional<EstimatedRobotPose> backPose = LumaHelpers.getPose(backCamera, kRobotToBackCam, "back");
    //     Optional<EstimatedRobotPose> leftPose = LumaHelpers.getPose(leftCamera, kRobotToLeftCam, "left");
    //     SmartDashboard.putBoolean("Front Pose Found", false);
    //     SmartDashboard.putBoolean("Left Pose Found", false);
    //     SmartDashboard.putBoolean("Back Pose Found", false);
    //     SmartDashboard.putBoolean("Blue Alliance", Mutables.blueAlliance);
    //     if (!frontPose.isEmpty()) {
    //         EstimatedRobotPose est = frontPose.get();
    //         frontPose2d = est.estimatedPose.toPose2d();
    //         frontTime = est.timestampSeconds;
    //         frontCamPose.setRobotPose(frontPose2d);
    //         SmartDashboard.putData("frontRobotPose", frontCamPose);
    //         SmartDashboard.putBoolean("Front Pose Found", true);
    //     }
    //     if (!backPose.isEmpty()) {
    //         EstimatedRobotPose est = backPose.get();
    //         backPose2d = est.estimatedPose.toPose2d();
    //         backTime = est.timestampSeconds;
    //         backCamPose.setRobotPose(backPose2d);
    //         SmartDashboard.putData("backRobotPose", backCamPose);
    //         SmartDashboard.putBoolean("Back Pose Found", true);
    //     }
    //     if (!leftPose.isEmpty()) {
    //         EstimatedRobotPose est = leftPose.get();
    //         leftPose2d = est.estimatedPose.toPose2d();
    //         leftTime = est.timestampSeconds;
    //         leftCamPose.setRobotPose(leftPose2d);
    //         SmartDashboard.putData("leftRobotPose", leftCamPose);
    //         SmartDashboard.putBoolean("Left Pose Found", true);
    //     }   

    //     getDistanceAndAverage(frontPose2d, frontTime, leftPose2d, leftTime, backPose2d, backTime, hubX, hubY);

    // }

    // public void getDistanceAndAverage(Pose2d frontPose, double frontTime, Pose2d leftPose, double leftTime, Pose2d backPose, double backTime, double hubX, double hubY) {

    //     double frontPoseX = 0;
    //     double frontPoseY = 0;
    //     double frontDist = 0;
    //     double frontYDiff = 0;
    //     double frontXDiff = 0;
    //     double frontHubAngle = 0;
    //     double leftPoseX = 0;
    //     double leftPoseY = 0;
    //     double leftDist = 0;
    //     double leftXDiff = 0;
    //     double leftYDiff = 0;
    //     double leftHubAngle = 0;
    //     double backPoseX = 0;
    //     double backPoseY = 0;
    //     double backDist = 0;
    //     double backXDiff = 0;
    //     double backYDiff = 0;
    //     double backHubAngle = 0;
    //     double avgDist = 0;
    //     double avgAngle = 0;
    //     double avgYDiff = 0;
    //     double camCount = 0;
    //     double angleDiff = 0;
        
    //     MechanismConstants.backTags = false;

    //     if (frontPose.getX() != -1) {
    //         frontPoseX = frontPose.getX();
    //         frontPoseY = frontPose.getY();
    //         frontXDiff = hubX - frontPoseX;
    //         frontYDiff = hubY - frontPoseY;
    //         frontDist = Math.sqrt( (frontXDiff * frontXDiff) + (frontYDiff * frontYDiff));
    //         frontHubAngle = Math.toDegrees(Math.atan(frontYDiff / (frontXDiff + 1E-6)));
    //         SmartDashboard.putNumber("Front Hub Angle", frontHubAngle);
    //         camCount++;
    //         Pose2d newFrontPose = new Pose2d(frontPoseX, frontPoseY, new Rotation2d(Math.toRadians(MechanismConstants.currentGyro)));
    //         drivetrain.addVisionMeasurement(newFrontPose, frontTime);
    //     }
    //     if (leftPose.getX() != -1) {
    //         leftPoseX = leftPose.getX();
    //         leftPoseY = leftPose.getY();
    //         leftXDiff = hubX - leftPoseX;
    //         leftYDiff = hubY - leftPoseY;
    //         leftDist = Math.sqrt( (leftXDiff * leftXDiff) + (leftYDiff * leftYDiff));
    //         leftHubAngle = Math.toDegrees(Math.atan(leftYDiff / (leftXDiff + 1E-6)));
    //         SmartDashboard.putNumber("Left Hub Angle", leftHubAngle);
    //         camCount++;
    //         Pose2d newLeftPose = new Pose2d(leftPoseX, leftPoseY, new Rotation2d(Math.toRadians(MechanismConstants.currentGyro)));
    //         drivetrain.addVisionMeasurement(newLeftPose, leftTime);
    //     }
    //     if (backPose.getX() != -1) {
    //         backPoseX = backPose.getX();
    //         backPoseY = backPose.getY();
    //         backXDiff = hubX - backPoseX;
    //         backYDiff = hubY - backPoseY;
    //         backDist = Math.sqrt( (backXDiff * backXDiff) + (backYDiff * backYDiff));
    //         backHubAngle = Math.toDegrees(Math.atan(backYDiff / (backXDiff + 1E-6)));
    //         SmartDashboard.putNumber("Back Hub Angle", backHubAngle);
    //         SmartDashboard.putNumber("Back xDiff", backXDiff);
    //         SmartDashboard.putNumber("Back yDiff", backYDiff);
    //         camCount++;
    //         MechanismConstants.backTags = true;
    //         Pose2d newBackPose = new Pose2d(backPoseX, backPoseY, new Rotation2d(Math.toRadians(MechanismConstants.currentGyro)));
    //         drivetrain.addVisionMeasurement(newBackPose, backTime);
    //     }

    //     // Average Distance Calculation
    //     if (camCount != 0) {
    //         avgDist = (frontDist + backDist + leftDist) / camCount;
    //         avgAngle = (frontHubAngle + backHubAngle + leftHubAngle) / camCount;
    //         avgYDiff = (frontYDiff + backYDiff + leftYDiff) / camCount;

    //         if (avgYDiff < 0) {
    //             avgAngle = 360 + avgAngle;
    //         }

    //         angleDiff = Math.abs(drivetrain.getCurrentGyro() - avgAngle);
    //         if (angleDiff <= MechanismConstants.gyroAccuracy) {
    //             MechanismConstants.linedUp = true;
    //         } else {
    //             MechanismConstants.linedUp = false;
    //         }

    //         if (Math.abs(MechanismConstants.targetYaw) <= .5) {
    //             MechanismConstants.yawLinedUp = true;
    //         } else {
    //             MechanismConstants.yawLinedUp = false;
    //         }
            
    //         if (((MechanismConstants.targetYaw >= 1) && (MechanismConstants.targetYaw <= 10)) || MechanismConstants.yawLinedUp) {
    //             MechanismConstants.yawLinedUp1 = true;
    //         } else {
    //             MechanismConstants.yawLinedUp1 = false;
    //         }

    //         if (((MechanismConstants.targetYaw >= 10) && (MechanismConstants.targetYaw <= 30)) || MechanismConstants.yawLinedUp) {
    //             MechanismConstants.yawLinedUp2 = true;
    //         } else {
    //             MechanismConstants.yawLinedUp2 = false;
    //         }

    //         if (((MechanismConstants.targetYaw >= 30) && (MechanismConstants.targetYaw <= 50)) || MechanismConstants.yawLinedUp) {
    //             MechanismConstants.yawLinedUp3 = true;
    //         } else {
    //             MechanismConstants.yawLinedUp3 = false;
    //         }

    //         if (((MechanismConstants.targetYaw <= -1) && (MechanismConstants.targetYaw >= -10)) || MechanismConstants.yawLinedUp) {
    //             MechanismConstants.yawLinedUp4 = true;
    //         } else {
    //             MechanismConstants.yawLinedUp4 = false;
    //         }

    //         if (((MechanismConstants.targetYaw <= -10) && (MechanismConstants.targetYaw >= -30)) || MechanismConstants.yawLinedUp) {
    //             MechanismConstants.yawLinedUp5 = true;
    //         } else {
    //             MechanismConstants.yawLinedUp5 = false;
    //         }

    //         if (((MechanismConstants.targetYaw <= -30) && (MechanismConstants.targetYaw >= -50)) || MechanismConstants.yawLinedUp) {
    //             MechanismConstants.yawLinedUp6 = true;
    //         } else {
    //             MechanismConstants.yawLinedUp6 = false;
    //         }

    //         MechanismConstants.canShoot = true;

    //     } else {

    //         MechanismConstants.canShoot = false;

    //     }

    //     MechanismConstants.targetDistance = avgDist;
    //     MechanismConstants.targetGyroAngle = avgAngle;
    //     SmartDashboard.putNumber("Average Hub Distance", avgDist);
    //     SmartDashboard.putNumber("Target Distance", MechanismConstants.targetDistance);
    //     SmartDashboard.putNumber("leftDist", leftDist);
    //     SmartDashboard.putNumber("frontDist", frontDist);
    //     SmartDashboard.putNumber("backDist", backDist);
    //     SmartDashboard.putNumber("leftX", leftPoseX);
    //     SmartDashboard.putNumber("leftY", leftPoseY);
    //     SmartDashboard.putNumber("backX", backPoseX);
    //     SmartDashboard.putNumber("backY", backPoseY);
    //     SmartDashboard.putNumber("frontX", frontPoseX);
    //     SmartDashboard.putNumber("frontY", frontPoseY);
    //     SmartDashboard.putNumber("hubX", hubX);
    //     SmartDashboard.putNumber("hubY", hubY);
    //     SmartDashboard.putNumber("Angle Diff", angleDiff);

    //     MechanismConstants.backX = backPoseX;
    //     MechanismConstants.backY = backPoseY;

    // }

}
