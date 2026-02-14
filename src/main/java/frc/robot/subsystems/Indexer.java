package frc.robot.subsystems;

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

/**
 * Define the Ball Indexer subsystem
 */
public class Indexer extends SubsystemBase{

  // Declare constants
  private final double MOTOR_DEADBAND = 0.001; // Deadband for the indexer motor. Values smaller than this will be rounded
                                               // to zero
    
  // Declare CAN ID for motor
  private final int indexerMotorID = 22;

  // Declare motor variable
  private TalonFX indexerMotor;

  // Declare Phoenix PID controller gains
  private double indexer_kG = 0.0;
  private double indexer_kS = 0.1;
  private double indexer_kV = 0.1;
  private double indexer_kA = 0.0;
  private double indexer_kP = 0.1;
  private double indexer_kI = 0.0;
  private double indexer_kD = 0.0;

  /** 
   * Create new indexer 
   */
  public Indexer(){

    //Create motors
    indexerMotor = new TalonFX(indexerMotorID, GeneralConstants.kMechBus);

    // Configure the motors
    InitializeMotor();

  }

  /**
   * Configure the motor
   */
  private void InitializeMotor(){

    // Create indexer motor configuration
    var indexerMotorConfigs = new TalonFXConfiguration();

    // Set indexer motor output configuration
    var indexerMotorOutputConfigs = indexerMotorConfigs.MotorOutput;
    indexerMotorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
    indexerMotorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
    indexerMotorOutputConfigs.withDutyCycleNeutralDeadband(MOTOR_DEADBAND);

    // Set indexer motor feedback sensor
    var indexerSensorConfig = indexerMotorConfigs.Feedback;
    indexerSensorConfig.withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);

    // Set indexer motor PID constants
    var slot0Configs = indexerMotorConfigs.Slot0;
    slot0Configs.kG = indexer_kG;
    slot0Configs.kS = indexer_kS;
    slot0Configs.kV = indexer_kV;
    slot0Configs.kA = indexer_kA;
    slot0Configs.kP = indexer_kP;
    slot0Configs.kI = indexer_kI;
    slot0Configs.kD = indexer_kD;

    // Apply indexer motor configuration and initialize position to 0
    StatusCode indexerMotorStatus = indexerMotor.getConfigurator().apply(indexerMotorConfigs, 0.050);
    if (!indexerMotorStatus.isOK()) {
      System.err.println("Could not apply indexer motor configs. Error code: " + indexerMotor.toString());
      DriverStation.reportError("Could not apply indexer motor configs.", false);
    } else {
      System.out.println("Successfully applied indexer motor configs. Error code: " + indexerMotorStatus.toString());
    }
    indexerMotor.getConfigurator().setPosition(0);

  }

  /**
   * Run the indexer motor at the specified output
   * 
   * @param speed  Output duty (-1 to 1) for the motor
   */
  public void runIndexer(double speed) {
    indexerMotor.setControl(new DutyCycleOut(speed));
  }

  /**
   * Halt indexer
   */
  public void stopIndexer() {
    indexerMotor.stopMotor();
  }
  
}