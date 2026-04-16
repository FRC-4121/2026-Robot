// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.MechanismState;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;
import frc.robot.Constants.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoShuttle extends Command {

  private Shooter myShooter;
  private Indexer myIndexer;

  private double percentVelocity;
  private double shooterVelocity;

  /** Creates a new AutoShuttle. */
  public AutoShuttle(Shooter shooter, Indexer indexer) {

    myShooter = shooter;
    myIndexer = indexer;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(myShooter, myIndexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    percentVelocity = .99;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    MechanismConstants.targetVelocity = 55;

    myShooter.runShooter(MechanismConstants.targetVelocity);
    shooterVelocity = myShooter.getShooterVelocity();

    if (Math.abs(shooterVelocity) > Math.abs(MechanismConstants.targetVelocity * percentVelocity)) {
      myIndexer.runIndexer(MechanismConstants.kIndexerSpeed);
      myIndexer.runFloor(MechanismConstants.kFloorSpeed);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    myShooter.stopShooter();
    myIndexer.stopIndexer();
    myIndexer.stopFloor();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return MechanismConstants.stopAutoShooter;
  }
}
