// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.MechanismConstants;
import frc.robot.subsystems.*;
import edu.wpi.first.units.measure.Velocity;
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

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoShoot extends Command {
  /** Creates a new AutoShoot. */
  private Shooter myShooter;
  private Indexer myIndexer;
  private double targetVelocity;
  private double percentVelocity;

    // Create new AutoShoot
    public AutoShoot(Shooter shooter, Indexer indexer) {

    myShooter = shooter;
    myIndexer = indexer;

    addRequirements(myShooter, myIndexer);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    percentVelocity = 0.95;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    myShooter.runShooter(targetVelocity);
    double shooterVelocity = myShooter.getShooterVelocity();

    if (shooterVelocity > percentVelocity * targetVelocity) {
      myIndexer.runIndexer(MechanismConstants.kIndexerSpeed);  
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
     myShooter.runShooter(0);
     myIndexer.runIndexer(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
