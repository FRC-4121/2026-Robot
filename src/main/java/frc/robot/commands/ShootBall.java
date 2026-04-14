// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import frc.robot.subsystems.*;
import frc.robot.RobotContainer;
import frc.robot.extras.Ballistics2026;
import frc.robot.generated.TunerConstants;
import edu.wpi.first.wpilibj2.command.Command;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.swerve.SwerveRequest.*;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.*;
import com.ctre.phoenix6.swerve.*;
import com.ctre.phoenix6.swerve.SwerveModule.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.MechanismConstants;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootBall extends Command {

  private Shooter myShooter;
  private Indexer myIndexer;
  private Intake myIntake;
  private CommandSwerveDrivetrain mySwerve;
  private double percentVelocity;
  private Ballistics2026 myBallistics;
  private final Pigeon2 myPigeon;

  //Park and idle swerve requests
  private final SwerveRequest.SwerveDriveBrake parkRequest = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();

  // ===Swerve Drive Variables===//
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  //Drive swerve request
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  private double hubDist;
  private double offset;
  private double output;
  private double speed;
  private double m_kP;
  private double m_kI;
  private double m_kD;

  private PIDController m_myPIDControl;

  /** Creates a new ShootBall. */
  public ShootBall(Shooter shooter, Indexer indexer, Intake intake, CommandSwerveDrivetrain swerve, Pigeon2 pigeon, Ballistics2026 ballistics) {

    myBallistics = ballistics;
    myShooter = shooter;
    myIndexer = indexer;
    myIntake = intake;
    mySwerve = swerve;
    myPigeon = pigeon;
    
    if (MechanismConstants.isShooterMode) {
      addRequirements(myShooter, myIndexer, myIntake, mySwerve);
    } else {
      addRequirements(myShooter, myIndexer, myIntake);
    }
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_kP = .04;
    m_kI = 0;
    m_kD = 0.0001;

    m_myPIDControl = new PIDController(m_kP, m_kI, m_kD);
    m_myPIDControl.setTolerance(0.1);

    percentVelocity = 0.99;
    MechanismConstants.hubDistance = MechanismConstants.targetDistance;
    mySwerve.setControl(idleRequest);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (MechanismConstants.isShooterMode) {

      if (MechanismConstants.canShoot) {

        offset = -MechanismConstants.targetYaw;
        output = m_myPIDControl.calculate(offset, 0);
        SmartDashboard.putNumber("Auto Rotate PID Output", output);

        MechanismConstants.targetVelocity = myBallistics.calculateLaunchVelcity(MechanismConstants.hubDistance,
        MechanismConstants.kShooterLaunchAngle, 
        MechanismConstants.kShooterSlip);

        myShooter.runShooter(MechanismConstants.targetVelocity);
        double shooterVelocity = myShooter.getShooterVelocity();
      SmartDashboard.putBoolean("Auto Rotate", false);
        if (MechanismConstants.isRotateEnabled) {
          SmartDashboard.putBoolean("Auto Rotate", true);
          SwerveRequest.FieldCentric driveRequest = new FieldCentric()
            .withVelocityX(0) // Drive forward with negative Y (forward)
            .withVelocityY(0) // Drive left with negative X (left)
            .withRotationalRate(output * MaxAngularRate); // Drive counterclockwise with negative X (left)
        

          mySwerve.setControl(driveRequest);
        }

        if ((Math.abs(shooterVelocity) > Math.abs(percentVelocity * MechanismConstants.targetVelocity)) && MechanismConstants.yawLinedUp) {
          mySwerve.setControl(parkRequest);
          myIndexer.runIndexer(MechanismConstants.kIndexerSpeed);
          myIndexer.runFloor(MechanismConstants.kFloorSpeed);
          myIntake.runShootingIntakeLift(MechanismConstants.kIntakeShootingPos);
        }

      } else {
        MechanismConstants.targetVelocity = 50;
        mySwerve.setControl(parkRequest);
        myShooter.runShooter(MechanismConstants.targetVelocity);
        double shooterVelocity = myShooter.getShooterVelocity();

        if ((Math.abs(shooterVelocity) > Math.abs(percentVelocity * MechanismConstants.targetVelocity))) {
          myIndexer.runIndexer(MechanismConstants.kIndexerSpeed);
          myIndexer.runFloor(MechanismConstants.kFloorSpeed);
          myIntake.runShootingIntakeLift(MechanismConstants.kIntakeShootingPos);
        }
      }

    } else {

      MechanismConstants.targetVelocity = 50;
      myShooter.runShooter(MechanismConstants.targetVelocity);
      double shooterVelocity = myShooter.getShooterVelocity();

      if (Math.abs(shooterVelocity) > Math.abs(percentVelocity * MechanismConstants.targetVelocity)) {
        myIndexer.runIndexer(MechanismConstants.kIndexerSpeed);
        myIndexer.runFloor(MechanismConstants.kFloorSpeed);
      }

    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    myShooter.stopShooter();
    myIndexer.stopIndexer();
    myIndexer.stopFloor();
    myIntake.runIntakeLift(MechanismConstants.kIntakeDown);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
