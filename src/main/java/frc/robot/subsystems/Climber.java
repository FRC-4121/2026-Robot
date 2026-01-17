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

public class Climber extends SubsystemBase {

  // Declare constants
  private final double DRIVE_DEADBAND = 0.001; // Deadband for the drive motor. Values smaller than this will be rounded
                                               // to zero

  // Declare CAN ID for motor
  private final int climberMotorID = 21;

  // Declare motor variables
  private TalonFX climberMotor;

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

  /** Creates a new Climber. */
  public Climber() {

    // Create motors
    climberMotor = new TalonFX(climberMotorID, GeneralConstants.CANBUS_NAME);

    // Create climber motor configuration
    var climberConfigs = new TalonFXConfiguration();

    // Set climber motor output configuration
    var climberOutputConfigs = climberConfigs.MotorOutput;
    climberOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
    climberOutputConfigs.NeutralMode = NeutralModeValue.Brake;
    climberOutputConfigs.withDutyCycleNeutralDeadband(DRIVE_DEADBAND);

    // Set climber motor feedback sensor
    var climberSensorConfig = climberConfigs.Feedback;
    climberSensorConfig.withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);

    // Set climber motor PID constants
    var slot0Configs = climberConfigs.Slot0;
    slot0Configs.kG = drive_kG;
    slot0Configs.kS = drive_kS;
    slot0Configs.kV = drive_kV;
    slot0Configs.kA = drive_kA;
    slot0Configs.kP = drive_kP;
    slot0Configs.kI = drive_kI;
    slot0Configs.kD = drive_kD;

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


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
