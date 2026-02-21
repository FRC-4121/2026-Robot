// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;

import frc.robot.subsystems.*;
import frc.robot.Constants.*;

/**
 * Define the ManualClimber command
 * 
 * This command moves the climber up or down in response to pushing the
 * left joystick on the auxilliary controller forward or back
 */
public class ManualClimber extends Command {

  /** Creates a new ManualClimber. */
  private Climber myClimber;
  private CommandXboxController myController;
  private double currentPosition;
  private double climbSpeed;
  private final SlewRateLimiter speedLimiter;
  
  /**
   * Create a new ManualClimber
   * 
   * @param climber An instance of a climber subsystem
   * @param controller An instance of a CommandXboxController
   */
  public ManualClimber(Climber climber, CommandXboxController controller) {
    
    // Initialize objects
    myClimber = climber;
    myController = controller;

    // Initialize speed limiter
    speedLimiter = new SlewRateLimiter(2);

    // Declare subsystem requirements for this command
    addRequirements(myClimber);
  }

  /**
   * Initialize the command
   * 
   * Called once when the command is initially scheduled
   */
  @Override
  public void initialize() {

    // Initialize variables
    climbSpeed = 0.0;

  }

  /**
   * Execute the commands actions
   * 
   * Called every time the scheduler runs while the command is scheduled
   */
  @Override
  public void execute() {

    // Get the joystick position and convert to speed
    climbSpeed = speedLimiter.calculate(MathUtil.applyDeadband(-myController.getRightY(), 0.01)) * ControlConstants.kJoystickSpeedCorr;

    // Get the current position of the climber
    currentPosition = myClimber.getPosition();

    // Only move the climber if not already at the extremes
    if (currentPosition > MechanismConstants.kClimberUp && climbSpeed < 0) {
      myClimber.manualRunClimber(0);
    } else if (currentPosition < MechanismConstants.kClimberDown && climbSpeed > 0) {
      myClimber.manualRunClimber(0);
    } else {
      myClimber.manualRunClimber(climbSpeed * MechanismConstants.kClimberSpeedFactor);
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

  }

  /**
   * Checks to see if command should end
   * 
   * Returns true when the command should end
   */
  @Override
  public boolean isFinished() {

    // Returning false means command will never end
    return false;
    
  }
}
