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

public class Climber extends SubsystemBase {

  // Declare constants
  private final double DRIVE_DEADBAND = 0.001; // Deadband for the drive motor. Values smaller than this will be rounded
                                               // to zero

  // Declare CAN ID for motor
  private final int climberLeadMotorID = 21;
  private final int climberFollowMotorID = 22;

  // Declare motor variables
  private TalonFX climberLeadMotor;
  private TalonFX climberFollowMotor;

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
    climberLeadMotor = new TalonFX(climberLeadMotorID, GeneralConstants.CANBUS_NAME);
    climberFollowMotor = new TalonFX(climberFollowMotorID, GeneralConstants.CANBUS_NAME);

   // Configure motors
   InitializeMotors();

   // Set follower to follow lead motor 
   climberFollowMotor.setControl(new Follower(climberLeadMotor.getDeviceID(), MotorAlignmentValue.Opposed));
   
  }

  /*
   * Configure the motors
   */
  private void InitializeMotors() {

    // Create lead climber motor configuration
    var climberLeadConfigs = new TalonFXConfiguration();


    // Set lead climber motor output configuration
    var climberLeadOutputConfigs = climberLeadConfigs.MotorOutput;
    climberLeadOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
    climberLeadOutputConfigs.NeutralMode = NeutralModeValue.Brake;
    climberLeadOutputConfigs.withDutyCycleNeutralDeadband(DRIVE_DEADBAND);

    

    // Set lead climber motor feedback sensor
    var climberSensorConfig = climberLeadConfigs.Feedback;
    climberSensorConfig.withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);

    // Set climber motor PID constants
    var slot0Configs = climberLeadConfigs.Slot0;
    slot0Configs.kG = drive_kG;
    slot0Configs.kS = drive_kS;
    slot0Configs.kV = drive_kV;
    slot0Configs.kA = drive_kA;
    slot0Configs.kP = drive_kP;
    slot0Configs.kI = drive_kI;
    slot0Configs.kD = drive_kD;

    // Apply lead climber motor configuration and initialize position to 0
    StatusCode climberStatus = climberLeadMotor.getConfigurator().apply(climberLeadConfigs, 0.050);
    if (!climberStatus.isOK()) {
      System.err.println("Could not apply climber motor configs. Error code: " + climberStatus.toString());
      DriverStation.reportError("Could not apply climber motor configs.", false);
    } else {
      System.out.println("Successfully applied drive motor configs. Error code: " + climberStatus.toString());
    }
    climberLeadMotor.getConfigurator().setPosition(0);




    // Create follower climber motor 
    var climberFollowConfigs = new TalonFXConfiguration();


    // Set follow climber motor output configuration
    var climberFollowOutputConfigs = climberFollowConfigs.MotorOutput;
    climberFollowOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
    climberFollowOutputConfigs.NeutralMode = NeutralModeValue.Brake;
    climberFollowOutputConfigs.withDutyCycleNeutralDeadband(DRIVE_DEADBAND);


    // Apply follower climber motor configuration and initialize position to 0
    StatusCode climberFollowStatus = climberFollowMotor.getConfigurator().apply(climberFollowConfigs, 0.050);
    if (!climberFollowStatus.isOK()) {
      System.err.println("Could not apply climber motor configs. Error code: " + climberFollowStatus.toString());
      DriverStation.reportError("Could not apply climber motor configs.", false);
    } else {
      System.out.println("Successfully applied drive motor configs. Error code: " + climberFollowStatus.toString());
    }
    climberFollowMotor.getConfigurator().setPosition(0);

  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
