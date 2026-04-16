// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;
import com.ctre.phoenix6.swerve.SwerveRequest.*;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.swerve.*;
import com.ctre.phoenix6.swerve.SwerveModule.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.MechanismConstants;
import frc.robot.subsystems.*;
import frc.robot.extras.Ballistics2026;
import frc.robot.generated.TunerConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoShoot extends Command {
  /** Creates a new AutoShoot. */
  private Shooter myShooter;
  private Indexer myIndexer;
  private Intake myIntake;
  private CommandSwerveDrivetrain mySwerve;
  private double percentVelocity;
  private double hubDist = 0;
  private Ballistics2026 myBallistics;
  
  private double offset;
  private double output;
  private double speed;
  private double m_kP;
  private double m_kI;
  private double m_kD;
  private double shooterVelocity;

  private PIDController m_myPIDControl;

  private final SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();

  // ===Swerve Drive Variables===//
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  //Drive swerve request
  private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors


    // Create new AutoShoot
    public AutoShoot(Shooter shooter, Indexer indexer, Intake intake, Ballistics2026 ballistics) {

    myBallistics = ballistics;
    myShooter = shooter;
    myIndexer = indexer;
    myIntake = intake;

    addRequirements(myShooter, myIndexer, myIntake);

  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    m_kP = .04;
    m_kI = 0;
    m_kD = 0.0001;

    m_myPIDControl = new PIDController(m_kP, m_kI, m_kD);
    m_myPIDControl.setTolerance(0.5);

    percentVelocity = 0.99;
    MechanismConstants.hubDistance = MechanismConstants.targetDistance;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

      if (MechanismConstants.canShoot) {

        MechanismConstants.targetVelocity = myBallistics.calculateLaunchVelcity(MechanismConstants.hubDistance,
        MechanismConstants.kShooterLaunchAngle, 
        MechanismConstants.kShooterSlip);

        myShooter.runShooter(MechanismConstants.targetVelocity);
        double shooterVelocity = myShooter.getShooterVelocity();

        if ((Math.abs(shooterVelocity) > Math.abs(percentVelocity * MechanismConstants.targetVelocity))) {
          myIndexer.runIndexer(MechanismConstants.kIndexerSpeed);
          myIndexer.runFloor(MechanismConstants.kFloorSpeed);
          myIntake.runShootingIntakeLift(MechanismConstants.kIntakeShootingPos);
        }


      } else {
        MechanismConstants.targetVelocity = 45;
        myShooter.runShooter(MechanismConstants.targetVelocity);
        shooterVelocity = myShooter.getShooterVelocity();

        if ((Math.abs(shooterVelocity) > Math.abs(percentVelocity * MechanismConstants.targetVelocity))) {
          myIndexer.runIndexer(MechanismConstants.kIndexerSpeed);
          myIndexer.runFloor(MechanismConstants.kFloorSpeed);
          myIntake.runShootingIntakeLift(MechanismConstants.kIntakeShootingPos);
        }
      }



  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    myShooter.stopShooter();
    myIntake.stopIntake();
    myIndexer.stopIndexer();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return MechanismConstants.stopAutoShooter;
  }
}
