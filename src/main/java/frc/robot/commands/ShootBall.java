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

    MechanismConstants.isIndexerMixing = false;
    percentVelocity = 0.95;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double hoodAngle = 0;
    double slipFactor = .25;

    if (MechanismConstants.isShooterMode) {

      if (MechanismConstants.numTagsFound != 0) {

        if (MechanismConstants.targetDistance > 2) {
          myShooter.runHood(MechanismConstants.kHoodHighPos);
          hoodAngle = 53;
          slipFactor = .3;
        } else {
          myShooter.runHood(MechanismConstants.kHoodLowPos);
          hoodAngle = 62.5;
          slipFactor = .23;
        }

        MechanismConstants.targetVelocity = -myBallistics.calculateLaunchVelcity(MechanismConstants.targetDistance,
            hoodAngle, slipFactor);
        myShooter.runShooter(MechanismConstants.targetVelocity);
        myIntake.runIntake(-0.5);
        double shooterVelocity = myShooter.getShooterVelocity();

        if ((Math.abs(shooterVelocity) > Math.abs(percentVelocity * MechanismConstants.targetVelocity)) || MechanismConstants.isIndexerOverride) {
          myIndexer.runIndexer(MechanismConstants.kIndexerSpeed);
        }

      } else {

        MechanismConstants.targetVelocity = -35;
        myShooter.runHood(MechanismConstants.kHoodLowPos);
        myShooter.runShooter(MechanismConstants.targetVelocity);
        myIntake.runIntake(-0.5);

        double shooterVelocity = myShooter.getShooterVelocity();

        if ((Math.abs(shooterVelocity) > Math.abs(percentVelocity * MechanismConstants.targetVelocity)) || MechanismConstants.isIndexerOverride) {
          myIndexer.runIndexer(MechanismConstants.kIndexerSpeed);
        }

      }

    } else {

      MechanismConstants.targetVelocity = -35;
      myShooter.runShooter(MechanismConstants.targetVelocity);
      double shooterVelocity = myShooter.getShooterVelocity();
      myIntake.runIntake(-0.5);
      myShooter.runHood(MechanismConstants.kHoodShuttlePos);

      if ((Math.abs(shooterVelocity) > Math.abs(percentVelocity * MechanismConstants.targetVelocity)) || MechanismConstants.isIndexerOverride) {
        myIndexer.runIndexer(MechanismConstants.kIndexerSpeed);
      }
      
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    myShooter.stopShooter();
    myIntake.stopIntake();
    myShooter.runHood(MechanismConstants.kHoodLowPos);
    //MechanismConstants.isIndexerMixing = true;
    myIndexer.stopIndexer();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
