// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;
import frc.robot.Constants.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LiftIntake extends Command {

  private Intake myIntake;

  private double currentPosition;
  private boolean done;

  private PIDController m_myPIDControl;

  /** Creates a new LiftIntake. */
  public LiftIntake(Intake intake) {
  
    myIntake = intake;
    addRequirements(myIntake);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_myPIDControl = new PIDController(MechanismConstants.kP_IntakeLift, MechanismConstants.kI_IntakeLift, MechanismConstants.kD_IntakeLift);
    m_myPIDControl.setTolerance(0.05);

    done = false;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    currentPosition = myIntake.getPosition();

    if (currentPosition > (MechanismConstants.kIntakeDown / 2)) {
      myIntake.runIntakeLift(MechanismConstants.kIntakeUp);
    } else {
      myIntake.runIntakeLift(MechanismConstants.kIntakeDown);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
