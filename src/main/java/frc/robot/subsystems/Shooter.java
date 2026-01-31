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

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.GeneralConstants;

public class Shooter extends SubsystemBase {
  
  

  // Declare CAN ID for motor
  private final int masterMotorID = 21;
  private final int slaveMotorID = 22;

  // Declare motor variables
  private TalonFX masterMotor;
  private TalonFX slaveMotor;

  // Declare Phoenix PID controller gains
  private double drive_kG = 0.0;
  private double drive_kS = 0.1;
  private double drive_kV = 0.1;
  private double drive_kA = 0.0;
  private double drive_kP = 0.1;
  private double drive_kI = 0.0;
  private double drive_kD = 0.0;

  // Declare motor output requests
  private final DutyCycleOut requestRotateDuty = new DutyCycleOut(0.0);
  private final DutyCycleOut requestShooterDuty = new DutyCycleOut(0.0);

  /** Creates a new Shooter. */
  public Shooter() {

    // Create motors
    masterMotor = new TalonFX(masterMotorID, GeneralConstants.CANBUS_NAME);
    slaveMotor = new TalonFX(slaveMotorID, GeneralConstants.CANBUS_NAME);

    slaveMotor.setControl(new Follower(masterMotorID, MotorAlignmentValue.Opposed));

    // Create shooter motor configuration
    var masterConfigs = new TalonFXConfiguration();
    var slaveConfigs = new TalonFXConfiguration();

    // Set shooter motor output configuration
    var masterOutputConfigs = masterConfigs.MotorOutput;
    masterOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
    masterOutputConfigs.NeutralMode = NeutralModeValue.Brake;
    masterOutputConfigs.withDutyCycleNeutralDeadband(DRIVE_DEADBAND);

    var slaveOutputConfigs = slaveConfigs.MotorOutput;
    slaveOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
    slaveOutputConfigs.NeutralMode = NeutralModeValue.Brake;
    slaveOutputConfigs.withDutyCycleNeutralDeadband(DRIVE_DEADBAND);

    // Set shooter motor feedback sensor
    var masterSensorConfig = masterConfigs.Feedback;
    masterSensorConfig.withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);

    // Set shooter motor PID constants
    var slot0Configs = masterConfigs.Slot0;
    slot0Configs.kG = drive_kG;
    slot0Configs.kS = drive_kS;
    slot0Configs.kV = drive_kV;
    slot0Configs.kA = drive_kA;
    slot0Configs.kP = drive_kP;
    slot0Configs.kI = drive_kI;
    slot0Configs.kD = drive_kD;

    // Apply shooter motor configuration and initialize position to 0
    StatusCode masterStatus = masterMotor.getConfigurator().apply(masterConfigs, 0.050);
    if (!masterStatus.isOK()) {
      System.err.println("Could not apply master motor configs. Error code: " + masterStatus.toString());
      DriverStation.reportError("Could not apply master motor configs.", false);
    } else {
      System.out.println("Successfully applied drive motor configs. Error code: " + masterStatus.toString());
    }
    masterMotor.getConfigurator().setPosition(0);

        StatusCode slaveStatus = slaveMotor.getConfigurator().apply(slaveConfigs, 0.050);
    if (!slaveStatus.isOK()) {
      System.err.println("Could not apply slave motor configs. Error code: " + slaveStatus.toString());
      DriverStation.reportError("Could not apply slave motor configs.", false);
    } else {
      System.out.println("Successfully applied drive motor configs. Error code: " + slaveStatus.toString());
    }
    slaveMotor.getConfigurator().setPosition(0);

  }

  /**
   * Runs shooter motors
   * @param speed speed and direction of the motor rotation (+ = clockwise)
   */
  public void runShooter(double speed) {
    masterMotor.setControl(new DutyCycleOut(speed));
  }

  /**
   * Halts shooter motors
   */
  public void stopShooter(){
    masterMotor.stopMotor();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Activate intake motor
   */
  public void setShooterSpeed(double shooterSpeed) {
  masterMotor.setControl(requestShooterDuty.withOutput(shooterSpeed));
  }

}
