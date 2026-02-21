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
import frc.robot.Constants.MechanismConstants;


/**
 * Define the climber subsystem
 */
public class Climber extends SubsystemBase {

  // Declare CAN ID for motor
  private final int climberMotorID = 26;
  
  // Declare motor variables
  private TalonFX climberMotor;
  
  // Declare Phoenix PID controller gains
  private double climb_kG = 0.0;
  private double climb_kS = 0.1;
  private double climb_kV = 0.1;
  private double climb_kA = 0.0;
  private double climb_kP = 0.1;
  private double climb_kI = 0.0;
  private double climb_kD = 0.0;

  /**
   * Create the climber subsystem
   */
  public Climber() {

    // Create motors
    climberMotor = new TalonFX(climberMotorID, GeneralConstants.kMechBus);

    // Configure motor
    InitializeMotor();

  }

  /*
   * Configure the climber motor
   */
  private void InitializeMotor() {

    // Create lead climber motor configuration
    var climberConfigs = new TalonFXConfiguration();

    // Set lead climber motor output configuration
    var climberOutputConfigs = climberConfigs.MotorOutput;
    climberOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
    climberOutputConfigs.NeutralMode = NeutralModeValue.Brake;
    climberOutputConfigs.withDutyCycleNeutralDeadband(MechanismConstants.kDutyCycleDeadband);

    // Set lead climber motor feedback sensor
    var climberSensorConfig = climberConfigs.Feedback;
    climberSensorConfig.withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);

    // Set climber motor PID constants
    var slot0Configs = climberConfigs.Slot0;
    slot0Configs.kG = climb_kG;
    slot0Configs.kS = climb_kS;
    slot0Configs.kV = climb_kV;
    slot0Configs.kA = climb_kA;
    slot0Configs.kP = climb_kP;
    slot0Configs.kI = climb_kI;
    slot0Configs.kD = climb_kD;

    // Apply climber motor configuration and initialize position to 0
    StatusCode climberStatus = climberMotor.getConfigurator().apply(climberConfigs, 0.050);
    if (!climberStatus.isOK()) {
      System.err.println("Could not apply climber motor configs. Error code: " + climberStatus.toString());
      DriverStation.reportError("Could not apply climber motor configs.", false);
    } else {
      System.out.println("Successfully applied drive motor configs. Error code: " + climberStatus.toString());
    }
    climberMotor.getConfigurator().setPosition(0);

  }

  /**
   * Run climber motor to a position
   * 
   * @param climberPos  Position target for the climber
   */
  public void runClimberToPos(double climberPos){
    climberMotor.setControl(new PositionVoltage(climberPos));
  }
  
  /**
   * Manually runs climber
   * 
   * @param value value to run the motor for in duty cycle out
   */
  public void manualRunClimber(double value){
    climberMotor.setControl(new DutyCycleOut(value));
  }

  /**
   * Gets current position of encoder 
   * 
   * @return The current encoder position
   */
  public double getPosition(){
    var rotorPosSignal = climberMotor.getRotorPosition();
    return rotorPosSignal.getValueAsDouble();
   }


  /**
  * Halt climber motor
  */
  public void stopClimber(){
    climberMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
