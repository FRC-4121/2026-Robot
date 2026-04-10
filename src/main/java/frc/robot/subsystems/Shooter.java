// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import org.photonvision.PhotonCamera;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.PositionDutyCycle;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//import org.photonvision.*;

import frc.robot.Constants.*;
import frc.robot.LumaHelpers;

/**
 * Define the Shooter subsystem
 */
public class Shooter extends SubsystemBase {
  
  // Declare CAN ID for motor
  private final int shooterMasterID = 25;
  private final int shooterSlaveID = 26;

  // Declare motor variables
  private TalonFX shooterMaster;
  private TalonFX shooterSlave;

  // Declare motor constants
  private final double MOTOR_DEADBAND = 0.05; // Deadband for the drive motor. Values smaller than this will be rounded
                                               // to zero

  /**
   * Create a new shooter
   */
  public Shooter() {

    // Create motors
    shooterMaster = new TalonFX(shooterMasterID, GeneralConstants.kMechBus);
    shooterSlave = new TalonFX(shooterSlaveID, GeneralConstants.kMechBus);

    // Initialize motors
    InitializeMotors();

    // Set slave motor to follow master motor
    shooterSlave.setControl(new Follower(shooterMaster.getDeviceID(), MotorAlignmentValue.Opposed));

  }

  /**
   * Initialize the shooter motors
   */
  public void InitializeMotors() {

    // Create shooter motor configuration
    var shooterMasterConfigs = new TalonFXConfiguration();

    // Set shooter motor output configuration
    var shooterMasterOutputConfigs = shooterMasterConfigs.MotorOutput;
    shooterMasterOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
    shooterMasterOutputConfigs.NeutralMode = NeutralModeValue.Coast;
    shooterMasterOutputConfigs.withDutyCycleNeutralDeadband(MOTOR_DEADBAND);

    // Set Current Limit Configuration
    var shooterMasterLimitConfigs = shooterMasterConfigs.CurrentLimits;
    shooterMasterLimitConfigs.StatorCurrentLimit = MechanismConstants.shooterCurrentLimit;
    shooterMasterLimitConfigs.StatorCurrentLimitEnable = true;

    // Set shooter motor feedback sensor
    var shooterSensorConfig = shooterMasterConfigs.Feedback;
    shooterSensorConfig.withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);

    // Set shooter motor PID constants
    var slot0Configs = shooterMasterConfigs.Slot0;
    slot0Configs.kG = MechanismConstants.kG_Shoot;
    slot0Configs.kS = MechanismConstants.kS_Shoot;
    slot0Configs.kV = MechanismConstants.kV_Shoot;
    slot0Configs.kA = MechanismConstants.kA_Shoot;
    slot0Configs.kP = MechanismConstants.kP_Shoot;
    slot0Configs.kI = MechanismConstants.kI_Shoot;
    slot0Configs.kD = MechanismConstants.kD_Shoot;

    // Set MotionMagic constants
    var motionMagicConfigs = shooterMasterConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = MechanismConstants.kMagicCruise;
    motionMagicConfigs.MotionMagicAcceleration = MechanismConstants.kMagicAccel;
    motionMagicConfigs.MotionMagicJerk = MechanismConstants.kMagicJerk;

    // Apply shooter motor configuration and initialize position to 0
    StatusCode shooterMasterStatus = shooterMaster.getConfigurator().apply(shooterMasterConfigs, 0.050);
    if (!shooterMasterStatus.isOK()) {
      System.err.println("Could not apply shooter motor configs. Error code: " + shooterMasterStatus.toString());
      DriverStation.reportError("Could not apply shooter motor configs.", false);
    } else {
      System.out.println("Successfully applied shooter motor configs. Error code: " + shooterMasterStatus.toString());
    }
    shooterMaster.getConfigurator().setPosition(0);



    // Create slave motor configuration
    var shooterSlaveConfigs = new TalonFXConfiguration();

    // Set slave motor output configuration
    var shooterSlaveOutputConfigs = shooterSlaveConfigs.MotorOutput;
    shooterSlaveOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
    shooterSlaveOutputConfigs.NeutralMode = NeutralModeValue.Coast;
    shooterSlaveOutputConfigs.withDutyCycleNeutralDeadband(MOTOR_DEADBAND);

    // Set Current Limit Configuration
    var shooterSlaveLimitConfigs = shooterSlaveConfigs.CurrentLimits;
    shooterSlaveLimitConfigs.StatorCurrentLimit = MechanismConstants.shooterCurrentLimit;
    shooterSlaveLimitConfigs.StatorCurrentLimitEnable = true;

    // Apply shooter slave motor configs and initialize position to 0
    StatusCode shooterSlaveStatus = shooterSlave.getConfigurator().apply(shooterSlaveConfigs, 0.050);
    if (!shooterSlaveStatus.isOK()) {
      System.err.println("Could not apply slave motor configs. Error code: " + shooterSlaveStatus.toString());
      DriverStation.reportError("Could not apply slave motor configs.", false);
    } else {
      System.out.println("Successfully applied slave motor configs. Error code: " + shooterSlaveStatus.toString());
    }
    shooterSlave.getConfigurator().setPosition(0);

  }

  /**
   * Runs shooter motor
   * @param speed speed and direction of the motor rotation (+ = clockwise)
   */
  public void runShooter(double speed) {
    shooterMaster.setControl(new MotionMagicVelocityVoltage(speed));
  }

  /**
   * Halts shooter motors
   */
  public void stopShooter(){
    shooterMaster.stopMotor();
  }

  /**
   * Returns shooter fly-wheel velocity in rotations per second (rps)
   * @return shooter velocity
   */
  public double getShooterVelocity() {

    return shooterMaster.getRotorVelocity().getValueAsDouble();
    
  }

/**
 * Get the current shooter motor velocity
 * @return
 */
public double getWheelVelocity() {

  return shooterMaster.getVelocity().getValueAsDouble();

}

/**
 * Get the current of the shooter motor
 * @return
 */
public double getShooterCurrent() {

  return shooterMaster.getStatorCurrent().getValueAsDouble();

}

@Override
public void periodic() {
  // This method will be called once per scheduler run
}

}
