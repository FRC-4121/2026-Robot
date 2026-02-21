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
 * Define a ManualLiftIntake command
 * 
 * This command moves the intake up or down in response to pushing the
 * left joystick on the auxilliary controller forward or back
 */
public class ManualLiftIntake extends Command {
 
  private Intake myIntake;
  private CommandXboxController myController;
  private double currentPosition;
  private final SlewRateLimiter speedLimiter;
  private double intakeSpeed;

 
  /**
   * Create a new ManualLiftIntake command
   * 
   * @param intake An instance of an intake subsystem
   * @param controller An instance of a CommandXboxController
   */
  public ManualLiftIntake(Intake intake, CommandXboxController controller) {

    // Initialize objects
    myIntake = intake;
    myController = controller;

    // Initialize speed limiter
    speedLimiter = new SlewRateLimiter(2);

    // Declare subsystem requirements for this command
    addRequirements(myIntake);

  }

  /**
   * Initialize the command
   * 
   * Called once when the command is initially scheduled
   */
  @Override
  public void initialize() {

    //Initialize variables
    intakeSpeed = 0.0;

  }

  /**
   * Execute the commands actions
   * 
   * Called every time the scheduler runs while the command is scheduled
   */
  @Override
  public void execute() {
   
    // Get the joystick position and convert to speed
    intakeSpeed = speedLimiter.calculate(MathUtil.applyDeadband(-myController.getLeftY(), 0.01)) * ControlConstants.kJoystickSpeedCorr;

    // Get current position of intake
    currentPosition = myIntake.getPosition();

    // Only move the intake if not already at the extremes
    if(currentPosition > MechanismConstants.kIntakeUp && intakeSpeed < 0) {
        myIntake.manualIntakeLift(0);
      } else if (currentPosition < MechanismConstants.kIntakeDown && intakeSpeed > 0) {
        myIntake.manualIntakeLift(0);
      } else {
        myIntake.manualIntakeLift(intakeSpeed * MechanismConstants.kIntakeSpeedFactor);
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
