// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.GeneralConstants;

/**
 * Define the Intake subsystem
 */
public class Intake extends SubsystemBase {

  // Declare constants
  private final double MOTOR_DEADBAND = 0.001; // Deadband for the drive motor. Values smaller than this will be rounded
                                               // to zero

  // Declare CAN ID for motor
  private final int intakeMotorID = 21;
  private final int intakeLiftID = 20;

  // Declare motor variables
  private TalonFX intakeMotor;
  private TalonFX intakeLift;

  // Declare Intake Phoenix PID controller gains
  private double intake_kG = 0.0;
  private double intake_kS = 0.1;
  private double intake_kV = 0.1;
  private double intake_kA = 0.0;
  private double intake_kP = 0.1;
  private double intake_kI = 0.0;
  private double intake_kD = 0.0;
  
  // Declare Lift Phoenix PID controller gains
  private double lift_kG = 0.0;
  private double lift_kS = 0.1;
  private double lift_kV = 0.1;
  private double lift_kA = 0.0;
  private double lift_kP = 0.1;
  private double lift_kI = 0.0;
  private double lift_kD = 0.0;

  /**
   * Create a new intake
   */
  public Intake() {

    // Create motors
    intakeMotor = new TalonFX(intakeMotorID, GeneralConstants.kMechBus);
    intakeLift = new TalonFX(intakeLiftID, GeneralConstants.kMechBus);

    // Initialize the motors
    InitializeMotors();

  }

  /**
   * Initialize the intake motors
   */
  private void InitializeMotors() {

    // Create intake roller motor configuration
    var intakeConfigs = new TalonFXConfiguration();

    // Set intake roller motor output configuration
    var intakeOutputConfigs = intakeConfigs.MotorOutput;
    intakeOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
    intakeOutputConfigs.NeutralMode = NeutralModeValue.Brake;
    intakeOutputConfigs.withDutyCycleNeutralDeadband(MOTOR_DEADBAND);

    // Set intake motor feedback sensor
    var intakeSensorConfig = intakeConfigs.Feedback;
    intakeSensorConfig.withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);

    // Set intake roller motor PID constants
    var slot0Configs = intakeConfigs.Slot0;
    slot0Configs.kG = intake_kG;
    slot0Configs.kS = intake_kS;
    slot0Configs.kV = intake_kV;
    slot0Configs.kA = intake_kA;
    slot0Configs.kP = intake_kP;
    slot0Configs.kI = intake_kI;
    slot0Configs.kD = intake_kD;

    // Apply intake roller motor configuration and initialize position to 0
    StatusCode intakeStatus = intakeMotor.getConfigurator().apply(intakeConfigs, 0.050);
    if (!intakeStatus.isOK()) {
      System.err.println("Could not apply intake motor configs. Error code: " + intakeStatus.toString());
      DriverStation.reportError("Could not apply intake motor configs.", false);
    } else {
      System.out.println("Successfully applied drive motor configs. Error code: " + intakeStatus.toString());
    }
    intakeMotor.getConfigurator().setPosition(0);


    // Create intake lift motor configuration
    var intakeLiftConfigs = new TalonFXConfiguration();

    // Set intake lift motor output configuration
    var intakeLiftOutputConfigs = intakeLiftConfigs.MotorOutput;
    intakeLiftOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
    intakeLiftOutputConfigs.NeutralMode = NeutralModeValue.Brake;
    intakeLiftOutputConfigs.withDutyCycleNeutralDeadband(MOTOR_DEADBAND);

    // Set intake lift motor feedback sensor
    var intakeLiftSensorConfig = intakeLiftConfigs.Feedback;
    intakeLiftSensorConfig.withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);

    // Set intake lift motor PID constants
    var slot0LiftConfigs = intakeLiftConfigs.Slot0;
    slot0LiftConfigs.kG = lift_kG;
    slot0LiftConfigs.kS = lift_kS;
    slot0LiftConfigs.kV = lift_kV;
    slot0LiftConfigs.kA = lift_kA;
    slot0LiftConfigs.kP = lift_kP;
    slot0LiftConfigs.kI = lift_kI;
    slot0LiftConfigs.kD = lift_kD;

    // Apply intake lift motor configuration and initialize position to 0
    StatusCode intakeLiftStatus = intakeLift.getConfigurator().apply(intakeLiftConfigs, 0.050);
    if (!intakeLiftStatus.isOK()) {
      System.err.println("Could not apply intake lift configs. Error code: " + intakeLiftStatus.toString());
      DriverStation.reportError("Could not apply intake lift configs.", false);
    } else {
      System.out.println("Successfully applied intake lift configs. Error code: " + intakeLiftStatus.toString());
    }
    intakeLift.getConfigurator().setPosition(0);

  }

  /**
   * Runs intake motor
   * 
   * @param speed speed and direction of the motor rotation (+ = clockwise)
   */
  public void runIntake(double speed) {
    intakeMotor.setControl(new DutyCycleOut(speed));
  }

  /**
   * Run intake lift motor
   * 
   * @param position desired position of the intake
   */
  public void runIntakeLift(double position) {
    intakeLift.setControl(new PositionVoltage(position));
  }

  /**
   * Halts intake motor
   */
  public void stopIntake(){
    intakeMotor.stopMotor();
  }

  /**
   * Halt inatke lift motor
   */
  public void stopIntakeLift(){
    intakeLift.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
