// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret;

/**
 * Define a new ManualTurret command
 * 
 * This command runs the turret left or right in response to pressing the
 * left or right bumper buttons on the main controller
 */
public class ManualTurret extends Command {

  private Turret myTurret;
  private double value;

  private double upperLimit;
  private double lowerLimit;
  private double currentPosition;

  /**
   * Create a new ManualTurret command
   * 
   * @param turret An instance of the turret subsystem
   * @param value The speed and direction to move the turret
   */
  public ManualTurret(Turret turret, double value) {

    // Initialize variables
    myTurret = turret;
    this.value = value;

    // Declare subsystem requirements for this command
    addRequirements(myTurret);

  }

  /**
   * Initialize the command
   * 
   * Called once when the command is initially scheduled
   */
  @Override
  public void initialize() {

  }

  /**
   * Execute the commands actions
   * 
   * Called every time the scheduler runs while the command is scheduled
   */
  @Override
  public void execute() {

    // Get the current position of the turret
    currentPosition = myTurret.getPosition();

    // Only move the turret if not already at the extremes
    if (currentPosition > upperLimit && value < 0) {
      myTurret.runTurret(0);
    } else if (currentPosition < lowerLimit && value > 0) {
      myTurret.runTurret(0);
    } else {
      myTurret.runTurret(value);
    }

  }

  /**
   * Ends the command
   * 
   * Called once the command ends or is interrupted
   * 
   * @param interrupted Flag to indicate if the command was interrupted
   */
  @Override
  public void end(boolean interrupted) {

    // Stop the turret from moving
    myTurret.stopTurret();

  }

  /**
   * Checks to see if command should end
   * 
   * Returns true when the command should end
   */
  @Override
  public boolean isFinished() {

    // Returning false means the command will not end
    return false;
    
  }
}
