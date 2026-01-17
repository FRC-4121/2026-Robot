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
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.GeneralConstants;

public class Shooter extends SubsystemBase {
  // Declare constants
  private final double DRIVE_DEADBAND = 0.001; // Deadband for the drive motor. Values smaller than this will be rounded
                                               // to zero

  // Declare CAN ID for motor
  private final int shooterMotorID = 21;

  // Declare motor variables
  private TalonFX shooterMotor;

  // Declare Phoenix PID controller gains
  private double drive_kG = 0.0;
  private double drive_kS = 0.1;
  private double drive_kV = 0.1;
  private double drive_kA = 0.0;
  private double drive_kP = 0.1;
  private double drive_kI = 0.0;
  private double drive_kD = 0.0;

  // Declare motor output requests
  private final PositionVoltage m_positionRequest = new PositionVoltage(0).withSlot(0);

  /** Creates a new Shooter. */
  public Shooter() {

    // Create motors
    shooterMotor = new TalonFX(shooterMotorID, GeneralConstants.CANBUS_NAME);

    // Create shooter motor configuration
    var shooterConfigs = new TalonFXConfiguration();

    // Set shooter motor output configuration
    var shooterOutputConfigs = shooterConfigs.MotorOutput;
    shooterOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
    shooterOutputConfigs.NeutralMode = NeutralModeValue.Brake;
    shooterOutputConfigs.withDutyCycleNeutralDeadband(DRIVE_DEADBAND);

    // Set shooter motor feedback sensor
    var shooterSensorConfig = shooterConfigs.Feedback;
    shooterSensorConfig.withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);

    // Set shooter motor PID constants
    var slot0Configs = shooterConfigs.Slot0;
    slot0Configs.kG = drive_kG;
    slot0Configs.kS = drive_kS;
    slot0Configs.kV = drive_kV;
    slot0Configs.kA = drive_kA;
    slot0Configs.kP = drive_kP;
    slot0Configs.kI = drive_kI;
    slot0Configs.kD = drive_kD;

    // Apply shooter motor configuration and initialize position to 0
    StatusCode shooterStatus = shooterMotor.getConfigurator().apply(shooterConfigs, 0.050);
    if (!shooterStatus.isOK()) {
      System.err.println("Could not apply shooter motor configs. Error code: " + shooterStatus.toString());
      DriverStation.reportError("Could not apply shooter motor configs.", false);
    } else {
      System.out.println("Successfully applied drive motor configs. Error code: " + shooterStatus.toString());
    }
    shooterMotor.getConfigurator().setPosition(0);

  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
