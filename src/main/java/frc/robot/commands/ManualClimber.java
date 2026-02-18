// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;
import frc.robot.Constants.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ManualClimber extends Command {

  /** Creates a new ManualClimber. */
  private Climber myClimber;
  private double currentPosition;
  private double value;
  

  public ManualClimber(Climber climber, double value) {
    myClimber = climber;
    this.value = value;

    addRequirements(myClimber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

      currentPosition = myClimber.getPosition();

      if (currentPosition > MechanismConstants.kClimberUp && value < 0) {
        myClimber.manualRunClimber(0);
      } else if (currentPosition < MechanismConstants.kClimberDown && value > 0) {
        myClimber.manualRunClimber(0);
      } else {
        myClimber.manualRunClimber(value * .5);
      }


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
