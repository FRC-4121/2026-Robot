package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GeneralConstants;
import org.photonvision.PhotonCamera;

/**
 * Define the Ball Indexer subsystem
 */
public class Indexer extends SubsystemBase{

  // Declare constants
  private final double MOTOR_DEADBAND = 0.001; // Deadband for the indexer motor. Values smaller than this will be rounded
                                               // to zero
    
  // Declare CAN ID for motor
  private final int indexerMasterID = 22;
  private final int indexerSlaveID = 23;


  // Declare motor variable
  private TalonFX indexerMaster;
  private TalonFX indexerSlave;

  private PhotonCamera hopperCam;

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
    indexerMaster = new TalonFX(indexerMasterID, GeneralConstants.kMechBus);
    indexerSlave = new TalonFX(indexerSlaveID, GeneralConstants.kMechBus);

    // Configure the motors
    InitializeMotor();

    // Set slave motor to follow master motor
    //indexerSlave.setControl(new Follower(indexerMaster.getDeviceID()), false));

    //Create a new indexer camera
    hopperCam = new PhotonCamera("hoppercam");

  }

  /**
   * Configure the motor
   */
  private void InitializeMotor(){

    // Create indexer motor configuration
    var indexerMasterConfigs = new TalonFXConfiguration();

    // Set indexer motor output configuration
    var indexerMasterOutputConfigs = indexerMasterConfigs.MotorOutput;
    indexerMasterOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
    indexerMasterOutputConfigs.NeutralMode = NeutralModeValue.Brake;
    indexerMasterOutputConfigs.withDutyCycleNeutralDeadband(MOTOR_DEADBAND);

    // Set indexer motor feedback sensor
    var indexerSensorConfig = indexerMasterConfigs.Feedback;
    indexerSensorConfig.withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);

    // Set indexer motor PID constants
    var slot0Configs = indexerMasterConfigs.Slot0;
    slot0Configs.kG = indexer_kG;
    slot0Configs.kS = indexer_kS;
    slot0Configs.kV = indexer_kV;
    slot0Configs.kA = indexer_kA;
    slot0Configs.kP = indexer_kP;
    slot0Configs.kI = indexer_kI;
    slot0Configs.kD = indexer_kD;

    // Apply indexer motor configuration and initialize position to 0
    StatusCode indexerMasterStatus = indexerMaster.getConfigurator().apply(indexerMasterConfigs, 0.050);
    if (!indexerMasterStatus.isOK()) {
      System.err.println("Could not apply indexer motor configs. Error code: " + indexerMaster.toString());
      DriverStation.reportError("Could not apply indexer motor configs.", false);
    } else {
      System.out.println("Successfully applied indexer motor configs. Error code: " + indexerMasterStatus.toString());
    }
    indexerMaster.getConfigurator().setPosition(0);




    // Create slave configuration
    var indexerSlaveConfigs = new TalonFXConfiguration();

    // Set indexer motor output configuration
    var indexerSlaveOutputConfigs = indexerMasterConfigs.MotorOutput;
    indexerSlaveOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
    indexerSlaveOutputConfigs.NeutralMode = NeutralModeValue.Brake;
    indexerSlaveOutputConfigs.withDutyCycleNeutralDeadband(MOTOR_DEADBAND);

  

    // Apply indexer motor configuration and initialize position to 0
    StatusCode indexerSlaveStatus = indexerSlave.getConfigurator().apply(indexerSlaveConfigs, 0.050);
    if (!indexerSlaveStatus.isOK()) {
      System.err.println("Could not apply indexer motor configs. Error code: " + indexerSlave.toString());
      DriverStation.reportError("Could not apply indexer motor configs.", false);
    } else {
      System.out.println("Successfully applied indexer motor configs. Error code: " + indexerSlaveStatus.toString());
    }
    indexerSlave.getConfigurator().setPosition(0);
    


  }

  /**
   * Run the indexer motor at the specified output
   * 
   * @param speed  Output duty (-1 to 1) for the motor
   */
  public void runIndexer(double speed) {
    indexerMaster.setControl(new DutyCycleOut(speed));
  }

  /**
   * Halt indexer
   */
  public void stopIndexer() {
    indexerMaster.stopMotor();
  }
  
}