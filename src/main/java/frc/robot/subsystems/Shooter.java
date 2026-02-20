// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.photonvision.*;

import frc.robot.Constants.GeneralConstants;

/**
 * Define the Shooter subsystem
 */
public class Shooter extends SubsystemBase {
  
  // Declare CAN ID for motor
  private final int shooterMotorID = 24;
  private final int hoodMotorID = 25;

  // Declare motor variables
  private TalonFX shooterMotor;
  private TalonFX hoodMotor;

  // Declare shooter camera
  PhotonCamera shooterCamera;



  // Declare shooter PID controller gains
  private double shooter_kG = 0.0;
  private double shooter_kS = 0.0;
  private double shooter_kV = 0.12;
  private double shooter_kA = 0.0;
  private double shooter_kP = 0.3;
  private double shooter_kI = 0.02;
  private double shooter_kD = 0.005;

  // Declare hood PID controller gains
  private double hood_kG = 0.0;
  private double hood_kS = 0.1;
  private double hood_kV = 0.1;
  private double hood_kA = 0.0;
  private double hood_kP = 0.1;
  private double hood_kI = 0.0;
  private double hood_kD = 0.0;

  // Declare MotionMagic variables
  private int magic_cruise = 100;
  private int magic_accel = 100;
  private int magic_jerk = 1500;

  // Declare motor constants
  private final double MOTOR_DEADBAND = 0.001; // Deadband for the drive motor. Values smaller than this will be rounded
                                               // to zero

  /**
   * Create a new shooter
   */
  public Shooter() {

    // Create motors
    shooterMotor = new TalonFX(shooterMotorID, GeneralConstants.kMechBus);
    hoodMotor = new TalonFX(hoodMotorID, GeneralConstants.kMechBus);

    // Create shooter camera
    shooterCamera = new PhotonCamera("shootercamera");

    // Initialize motors
    InitializeMotors();

  }

  /**
   * Initialize the shooter motors
   */
  public void InitializeMotors() {

    // Create shooter motor configuration
    var shooterConfigs = new TalonFXConfiguration();

    // Set shooter motor output configuration
    var shooterOutputConfigs = shooterConfigs.MotorOutput;
    shooterOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
    shooterOutputConfigs.NeutralMode = NeutralModeValue.Brake;
    shooterOutputConfigs.withDutyCycleNeutralDeadband(MOTOR_DEADBAND);

    // Set shooter motor feedback sensor
    var shooterSensorConfig = shooterConfigs.Feedback;
    shooterSensorConfig.withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);

    // Set shooter motor PID constants
    var slot0Configs = shooterConfigs.Slot0;
    slot0Configs.kG = shooter_kG;
    slot0Configs.kS = shooter_kS;
    slot0Configs.kV = shooter_kV;
    slot0Configs.kA = shooter_kA;
    slot0Configs.kP = shooter_kP;
    slot0Configs.kI = shooter_kI;
    slot0Configs.kD = shooter_kD;

    // Set MotionMagic constants
    var motionMagicConfigs = shooterConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = magic_cruise;
    motionMagicConfigs.MotionMagicAcceleration = magic_accel;
    motionMagicConfigs.MotionMagicJerk = magic_jerk;

    // Apply shooter motor configuration and initialize position to 0
    StatusCode shooterStatus = shooterMotor.getConfigurator().apply(shooterConfigs, 0.050);
    if (!shooterStatus.isOK()) {
      System.err.println("Could not apply shooter motor configs. Error code: " + shooterStatus.toString());
      DriverStation.reportError("Could not apply shooter motor configs.", false);
    } else {
      System.out.println("Successfully applied shooter motor configs. Error code: " + shooterStatus.toString());
    }
    shooterMotor.getConfigurator().setPosition(0);

    // Create hood motor configuration
    var hoodConfigs = new TalonFXConfiguration();

    // Set hood motor output configuration
    var hoodOutputConfigs = hoodConfigs.MotorOutput;
    hoodOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
    hoodOutputConfigs.NeutralMode = NeutralModeValue.Brake;
    hoodOutputConfigs.withDutyCycleNeutralDeadband(MOTOR_DEADBAND);

    // Set hood motor feedback sensor
    var hoodSensorConfig = hoodConfigs.Feedback;
    hoodSensorConfig.withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);

    // Set hood motor PID constants
    var hoodSlot0Configs = hoodConfigs.Slot0;
    hoodSlot0Configs.kG = hood_kG;
    hoodSlot0Configs.kS = hood_kS;
    hoodSlot0Configs.kV = hood_kV;
    hoodSlot0Configs.kA = hood_kA;
    hoodSlot0Configs.kP = hood_kP;
    hoodSlot0Configs.kI = hood_kI;
    hoodSlot0Configs.kD = hood_kD;

    // Apply hood motor configs and initialize position to 0
    StatusCode hoodStatus = hoodMotor.getConfigurator().apply(hoodConfigs, 0.050);
    if (!hoodStatus.isOK()) {
      System.err.println("Could not apply hood motor configs. Error code: " + hoodStatus.toString());
      DriverStation.reportError("Could not apply hood motor configs.", false);
    } else {
      System.out.println("Successfully applied hood motor configs. Error code: " + hoodStatus.toString());
    }
    hoodMotor.getConfigurator().setPosition(0);

  }

  /**
   * Runs shooter motor
   * @param speed speed and direction of the motor rotation (+ = clockwise)
   */
  public void runShooter(double speed) {
    shooterMotor.setControl(new MotionMagicVelocityVoltage(speed));
  }

  /**
   * Halts shooter motors
   */
  public void stopShooter(){
    shooterMotor.stopMotor();
  }

  /**
   * Returns shooter fly-wheel velocity in rotations per second (rps)
   * @return shooter velocity
   */
  public double getShooterVelocity() {

    return shooterMotor.getRotorVelocity().getValueAsDouble();
    
  }

  /**
   *  Runs hood motor
   */
  public void runHood(double position) {
    hoodMotor.setControl(new PositionVoltage(position));
  }

 /**
 * Halts hood motor
 */
public void stopHood() {
  hoodMotor.stopMotor();
}

/**
 * Get position from hood
 * @return hood position
 */
public double getHoodPosition() {

  return hoodMotor.getPosition().getValueAsDouble();

}

/**
 * Get yaw 
 */


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }



}
