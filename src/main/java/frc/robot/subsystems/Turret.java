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
import frc.robot.LimelightHelpers;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.*;

/**
 * Define the Turret subsystem
 */
public class Turret extends SubsystemBase {

  // Declare constants
  private final double MOTOR_DEADBAND = 0.001; // Deadband for the drive motor. Values smaller than this will be rounded
                                               // to zero

  // Declare CAN ID for motor
  private final int turretMotorID = 23;

  // Declare motor variables
  private TalonFX turretMotor;

  /**
   * Create a new Turret
   */
  public Turret() {

    // Create a new turret motor
    turretMotor =  new TalonFX(turretMotorID, GeneralConstants.kMechBus);

    // Initialize the motor
    InitializeMotor();

  }

  /**
   * Initialize the Turret motor
   */
  private void InitializeMotor() {

    // Create indexer motor configuration
    var turretMotorConfigs = new TalonFXConfiguration();

    // Set indexer motor output configuration
    var turretMotorOutputConfigs = turretMotorConfigs.MotorOutput;
    turretMotorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
    turretMotorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
    turretMotorOutputConfigs.withDutyCycleNeutralDeadband(MOTOR_DEADBAND);

    // Set indexer motor feedback sensor
    var turretSensorConfig = turretMotorConfigs.Feedback;
    turretSensorConfig.withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);

    // Apply indexer motor configuration and initialize position to 0
    StatusCode turretMotorStatus = turretMotor.getConfigurator().apply(turretMotorConfigs, 0.050);
    if (!turretMotorStatus.isOK()) {
      System.err.println("Could not apply indexer motor configs. Error code: " + turretMotor.toString());
      DriverStation.reportError("Could not apply indexer motor configs.", false);
    } else {
      System.out.println("Successfully applied indexer motor configs. Error code: " + turretMotorStatus.toString());
    }
    turretMotor.getConfigurator().setPosition(0);

  }

  /**
   * Runs turret motor
   * 
   * @param speed speed and direction of the motor rotation (+ = clockwise)
   */
  public void runTurret(double speed) {
    turretMotor.setControl(new DutyCycleOut(speed));
  }

  /**
   * Halts turret motor
   */
  public void stopTurret(){
    turretMotor.stopMotor();
  }

  /**
   * Gets current position of encoder 
   * 
   * @return The current encoder position
   */
  public double getPosition(){
    var rotorPosSignal = turretMotor.getRotorPosition();
    return rotorPosSignal.getValueAsDouble();
   }

  /**
   * Reset position to 0
   */
  public void resetPosition(){
    turretMotor.getConfigurator().setPosition(0);
  }

  /**
   * Get the current target offset from the turret camera
   * 
   * @return The current target offset
   */
  public double GetOffset() {
    return LimelightHelpers.getTX("limelight-turret");
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("turret Pos", getPosition());

  }

}