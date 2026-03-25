// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;
import frc.robot.Constants.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MixHopper extends Command {

  private Indexer myIndexer;

  /** Creates a new MixHopper. */
  public MixHopper(Indexer indexer) {

    myIndexer = indexer;
    addRequirements(myIndexer);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    myIndexer.runIndexer(-.3);

    // if (MechanismConstants.isIndexerMixing) {

    //   myIndexer.runIndexer(-.3);
    //   System.out.println("Mixing Hopper");

    // }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    myIndexer.stopIndexer();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
