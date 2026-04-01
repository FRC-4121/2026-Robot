// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GeneralConstants;

public class Floor extends SubsystemBase {
  
  // Declare constants
  private final double MOTOR_DEADBAND = 0.001;

  // Declare CAN ID for the motor
  private final int indexerFloorID = 24;

  // Declare motor variable
  private TalonFX indexerFloor;


  public Floor() {

    // Create motor
    indexerFloor = new TalonFX(indexerFloorID, GeneralConstants.kMechBus);

    // Configure motor
    InitializeMotor();

  }

  /**
   * Configure the motor
   */
    private void InitializeMotor(){

      // // Create floor motor configuration
    var indexerFloorConfigs = new TalonFXConfiguration();

    // Set floor motor output configuration
    var indexerFloorOutputConfigs = indexerFloorConfigs.MotorOutput;
    indexerFloorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
    indexerFloorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
    indexerFloorOutputConfigs.withDutyCycleNeutralDeadband(MOTOR_DEADBAND);


  

    // Apply indexer motor configuration and initialize position to 0
    StatusCode indexerFloorStatus = indexerFloor.getConfigurator().apply(indexerFloorConfigs, 0.050);
    if (!indexerFloorStatus.isOK()) {
      System.err.println("Could not apply indexer motor configs. Error code: " + indexerFloor.toString());
      DriverStation.reportError("Could not apply indexer motor configs.", false);
    } else {
      System.out.println("Successfully applied indexer motor configs. Error code: " + indexerFloorStatus.toString());
    }
    indexerFloor.getConfigurator().setPosition(0);
    }



    
    /**
     * Run the indexer floor at a specified output
     * @param speed
     */
    public void runFloor(double speed){
      indexerFloor.setControl(new DutyCycleOut(speed));
    }

    /**
     * Halt the indexer floor motor
     */
    public void stopFloor(){
      indexerFloor.stopMotor();
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
