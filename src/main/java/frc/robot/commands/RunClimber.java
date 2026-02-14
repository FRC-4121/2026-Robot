// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;
import frc.robot.Constants.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunClimber extends Command {

  private Climber myClimber;
  private double climberPos;

  /** Creates a new LiftClimber. */
  public RunClimber(Climber climber) {

    myClimber = climber;
    addRequirements(myClimber);
    

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    if(MechanismConstants.isClimberUp){
      MechanismConstants.isClimberUp = false;
    } else {
      MechanismConstants.isClimberUp = true;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(MechanismConstants.isClimberUp) {
      climberPos = MechanismConstants.kClimberUp;
    } else {
      climberPos = MechanismConstants.kClimberDown;
    }

    myClimber.runClimber(climberPos);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
    myClimber.stopClimber();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
