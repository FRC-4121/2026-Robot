// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.*;
import frc.robot.extras.Ballistics2026;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.math.*;

import frc.robot.Constants.MechanismConstants;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootBall extends Command {

  private Shooter myShooter;
  private Indexer myIndexer;
  private double percentVelocity;
  private Ballistics2026 myBallistics;

  /** Creates a new ShootBall. */
  public ShootBall(Shooter shooter, Indexer indexer) {

    myShooter = shooter;
    myIndexer = indexer;

    addRequirements(myShooter, myIndexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    percentVelocity = 0.9;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double hoodAngle = 0;

    if (MechanismConstants.isShooterMode) {

      if (MechanismConstants.targetDistance > 2) {
        myShooter.runHood(MechanismConstants.kHoodHighPos);
        hoodAngle = 50; //This is a guess. Confirm this with a measured value
      } else {
        myShooter.runHood(MechanismConstants.kHoodLowPos);
        hoodAngle = 65;
      }

      // myBallistics = new Ballistics2026(hoodAngle, 
      //                                 MechanismConstants.kTurretCameraHeight, 
      //                                 MechanismConstants.kTargetHeight, 
      //                                 MechanismConstants.kShooterSlip,
      //                                 MechanismConstants.kShooterWheelDiameter,
      //                                 MechanismConstants.kShooterDriveRatio);

      // MechanismConstants.targetVelocity = myBallistics.calculateLaunchVelcity(MechanismConstants.targetDistance);
      MechanismConstants.targetVelocity = -32.5;
      SmartDashboard.putNumber("Target Velocity", MechanismConstants.targetVelocity);
      myShooter.runShooter(MechanismConstants.targetVelocity);
      double shooterVelocity = myShooter.getShooterVelocity();

      if (Math.abs(shooterVelocity) > Math.abs(percentVelocity * MechanismConstants.targetVelocity)) {
        myIndexer.runIndexer(MechanismConstants.kIndexerSpeed);
      }

    } else {

      MechanismConstants.targetVelocity = -80;
      myShooter.runShooter(MechanismConstants.targetVelocity);
      double shooterVelocity = myShooter.getShooterVelocity();
      myShooter.runHood(MechanismConstants.kHoodShuttlePos);

      if (Math.abs(shooterVelocity) > Math.abs(percentVelocity * MechanismConstants.targetVelocity)) {
        myIndexer.runIndexer(MechanismConstants.kIndexerSpeed);
      }
      
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    myShooter.stopShooter();
    myIndexer.stopIndexer();
    myShooter.runHood(MechanismConstants.kHoodLowPos);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
