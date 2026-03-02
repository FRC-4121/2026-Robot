// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ZeroEncoders extends Command {

  Intake myIntake;
  Turret myTurret;
  Climber myClimber;

  /** Creates a new ZeroEncoder. */
  public ZeroEncoders(Intake intake, Turret turret, Climber climber) {
    
    myIntake = intake;
    myTurret = turret;
    myClimber = climber;

    addRequirements(myIntake, myTurret, myClimber);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    myIntake.zeroIntakeLift();
    myTurret.zeroTurret();
    myClimber.zeroClimber();

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
