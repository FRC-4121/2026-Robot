// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.MechanismConstants;
import frc.robot.subsystems.*;
import frc.robot.extras.Ballistics2026;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoShoot extends Command {
  /** Creates a new AutoShoot. */
  private Shooter myShooter;
  private Indexer myIndexer;
  private Intake myIntake;
  private double percentVelocity;
  private double hubDist = 0;
  private Ballistics2026 myBallistics;

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
    
    percentVelocity = 0.95;
    MechanismConstants.stopAutoShooter = false;
    hubDist = MechanismConstants.targetDistance;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

            MechanismConstants.targetVelocity = myBallistics.calculateLaunchVelcity(hubDist,
            MechanismConstants.kShooterLaunchAngle, MechanismConstants.kShooterSlip);
        myShooter.runShooter(MechanismConstants.targetVelocity);
        double shooterVelocity = myShooter.getShooterVelocity();

        // if (MechanismConstants.isRotateEnabled) {
        //   SwerveRequest.FieldCentricFacingAngle angleRequest = new FieldCentricFacingAngle()
        //       .withVelocityX(0)
        //       .withVelocityY(0)
        //       .withTargetDirection(new Rotation2d(MechanismConstants.targetGyroAngle));
        //   mySwerve.setControl(angleRequest);
        // }

        if ((Math.abs(shooterVelocity) > Math.abs(percentVelocity * MechanismConstants.targetVelocity))) {
          myIndexer.runIndexer(MechanismConstants.kIndexerSpeed);
          myIndexer.runFloor(MechanismConstants.kFloorSpeed);
          myIntake.runShootingIntakeLift(MechanismConstants.kIntakeShootingPos);
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
