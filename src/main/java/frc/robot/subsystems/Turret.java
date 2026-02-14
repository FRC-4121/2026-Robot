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
import frc.robot.LimelightHelpers;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.GeneralConstants;

public class Turret extends SubsystemBase {
  // Declare constants
  private final double MOTOR_DEADBAND = 0.001; // Deadband for the drive motor. Values smaller than this will be rounded
                                               // to zero

  // Declare CAN ID for motor
  private final int turretMotorID = 19;

  // Declare motor variables
  private TalonFX turretMotor;

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
  private final DutyCycleOut requestTurretDuty = new DutyCycleOut(0.0);
  

  /** Creates a new Turret. */
  public Turret() {


  }

    /**
   * Runs turret motor
   * @param speed speed and direction of the motor rotation (+ = clockwise)
   */
  public void runturret(double speed) {
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
   * @return
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

  @Override
  public void periodic() {
    SmartDashboard.putNumber("turret Pos", getPosition());

  }

  /**
   * Activate turret motor
   */
  public void setTurretSpeed(double turretSpeed) {
    turretMotor.setControl(requestTurretDuty.withOutput(turretSpeed));
  }

  public double GetOffset() {
    return LimelightHelpers.getTX("limelight-turret");
  }


}