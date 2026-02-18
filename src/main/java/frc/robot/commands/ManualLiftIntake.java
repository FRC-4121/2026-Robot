// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.*;
import frc.robot.Constants.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ManualLiftIntake extends Command {
 
 private Intake myIntake;
 private double currentPosition;
 private double value;
 
  /** Creates a new ManualLiftIntake. */
  public ManualLiftIntake(Intake intake, double value) {

    myIntake = intake;
    this.value = value;
    
    addRequirements(myIntake);


  }

// Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   
    currentPosition = myIntake.getPosition();

    if(currentPosition > MechanismConstants.kIntakeUp && value < 0) {
        myIntake.manualIntakeLift(0);
      } else if (currentPosition < MechanismConstants.kIntakeDown && value > 0) {
        myIntake.manualIntakeLift(0);
      } else {
        myIntake.manualIntakeLift(value * .1);
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
