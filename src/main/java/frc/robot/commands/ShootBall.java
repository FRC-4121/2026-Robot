// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.*;
import frc.robot.extras.Ballistics2026;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.MechanismConstants;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootBall extends Command {

  private Shooter myShooter;
  private Indexer myIndexer;
  private Intake myIntake;
  private double percentVelocity;
  private Ballistics2026 myBallistics;

  private double hubDist;  

  /** Creates a new ShootBall. */
  public ShootBall(Shooter shooter, Indexer indexer, Intake intake, Ballistics2026 ballistics) {

    myBallistics = ballistics;
    myShooter = shooter;
    myIndexer = indexer;
    myIntake = intake;
    addRequirements(myShooter, myIndexer, myIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    percentVelocity = 0.99;
    hubDist = MechanismConstants.targetDistance;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (MechanismConstants.isShooterMode) {

      MechanismConstants.targetVelocity = myBallistics.calculateLaunchVelcity(hubDist, MechanismConstants.kShooterLaunchAngle, MechanismConstants.kShooterSlip);
      myShooter.runShooter(MechanismConstants.targetVelocity);
      double shooterVelocity = myShooter.getShooterVelocity();

      if (Math.abs(shooterVelocity) > Math.abs(percentVelocity * MechanismConstants.targetVelocity)) {
        myIndexer.runIndexer(MechanismConstants.kIndexerSpeed);
        myIndexer.runFloor(MechanismConstants.kFloorSpeed);
        myIntake.runShootingIntakeLift(MechanismConstants.kIntakeShootingPos);
      }

    } else {

      MechanismConstants.targetVelocity = -25;
      myShooter.runShooter(MechanismConstants.targetVelocity);
      double shooterVelocity = myShooter.getShooterVelocity();

      if (Math.abs(shooterVelocity) > Math.abs(percentVelocity * MechanismConstants.targetVelocity)) {
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
