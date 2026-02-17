// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ManualTurret extends Command {

  private Turret myTurret;
  private double value;

  private double upperLimit;
  private double lowerLimit;
  private double currentPosition;

  /** Creates a new runTurret. */
  public ManualTurret(Turret turret, double value) {

    myTurret = turret;
    this.value = value;

    addRequirements(myTurret);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

      currentPosition = myTurret.getPosition();

      if (currentPosition > upperLimit && value < 0) {
        myTurret.runTurret(0);
      } else if (currentPosition < lowerLimit && value > 0) {
        myTurret.runTurret(0);
      } else {
        myTurret.runTurret(value);
      }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  myTurret.stopTurret();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
