/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.pathplanner.lib.config.PIDConstants;

/**
 * The Constants class provides a dope af place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be
 * declared globally (i.e. public static). Do not put anything functional in
 * this class.
 *
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */

public final class Constants {

  /*
   * Values used for driving
   */
  public static final class DriveConstants {

    /*
     * Swerve drive values
     */

    // Swerve drive calculation constants
    public static final double kTalonFXPPR = 2048;
    public static final double leftGyroCorrection = 53;// Gyro correction in degrees (was 53)
    public static final double autoLeftGyroCorrection = 53;
    public static final double rightGyroCorrection = 307;// Gyro correction in degrees (was 307)
    public static final double autoRightGyroCorrection = -53;
    public static final double kGearRatio = 7.15;

    // Swerve drive PID constants
    public static final double kAnglePIDkp = 0.0145;
    public static final double kAnglePIDkd = 0.0001;
    public static final double kAnglePIDki = 0.0;
    public static final double drivePIDkPs[] = { 0.4, 0.4, 0.4, 0.4 };
    public static final double drivePIDkIs[] = { 0.25, 0.25, 0.25, 0.25 };
    public static final double drivePIDkDs[] = { 0.0, 0.0, 0.0, 0.0 };
    public static final double drivePIDkFs[] = { 1.0, 1.0, 1.0, 1.0 };
    public static final double anglePIDkPs[] = { 3.25, 3.25, 3.25, 3.25 }; // 3.25
    public static final double anglePIDkIs[] = { 2.25, 2.25, 2.25, 2.25 }; // 2.25
    public static final double anglePIDkDs[] = { 0.04, 0.04, 0.04, 0.04 }; // 0.04
    public static double angleLimiters[] = { 1.0, 1.0, 1.0, 1.0 };

    // Auto drive PID constants
    public static final double FastMaxYawRate = 0.2;
    public static final double SlowMaxYawRate = 0.35;
    public static double maxYawRate = 0.2;
    public static final double kP_DriveAngle = 0.01;
    public static final double kI_DriveAngle = 0.0;
    public static final double kD_DriveAngle = 0.0;

    public static boolean AutoAngleToTarget = false;

    // Collision constants
    public static final double kCollisionThresholdDeltaG = 80000;

    // Motor Limits
    public static final double MaxLinearSpeed = 3.7; // 3.7 Max Speed in Meters per second
    public static final double MaxRadiansPerSecond = Math.PI;
    public static final double SlowMaxLinearSpeed = 0.75;// 0.75 Max Speed during slow mode in meters per second
    public static final double SlowRadiansPerSecond = (Math.PI / 2);// Max rotational speed during slow mode
    public static final double swerveDriveSpeedLimiter = 0.7;
    public static double LinearSpeed = 3.7;
    public static double RotationalSpeed = Math.PI;
    public static double RotationalSpeedFast = 2 * Math.PI;

    // PathPlanner constants
    public static final PIDConstants translationConstants = new PIDConstants(4.5, 0.0, 0.0);
    public static final PIDConstants rotationConstants = new PIDConstants(1.5, 0.0, 0.0);

    // General drive constants
    public static final double GyroCorrection = 0;

  }

  /*
   * Values used for mechanisms
   */
  public static final class MechanismConstants {

    // Set climber constants
    public static final double kClimberDown = 0;
    public static final double kClimberUp = 100; //Edit this to be the encoder vaue when the climber is raised
    public static final int CLIMBER_LIMIT_SWITCH = 2;
    public static boolean isClimberUp = true;
    
    // Set shooter constants
    public static final int SHOOTER_LEAD = 1;
    public static final int SHOOTER_FOLLOW = 0;

    public static final double kP_Shoot = 0.00015; //was 0.1
    public static final double kI_Shoot = 0.0000;
    public static final double kD_Shoot = 0;
    public static final double kF_Shoot = -1;

    public static final int kPIDLoopIdxShoot = 0;
    public static final int kTimeoutMsShoot = 20;
    public static final int kShooterMaxRPM = 6100;

    public static final double kShooterSpeed = -1.0;

    // Set hopper/indexer constants

   
    //Set intake constants
    public static final double kIntakeUp = 0;
    public static final double kIntakeDown = 100; //Edit this to be the encoder value when the intake is lowered
    public static boolean isIntakeUp = true;

    //Set turret constants
    public static final int TURRET = 9;

    public static final double kTalonFXPPR = 2048;
    public static final double kTurretEncoderPPR = 10.5;
    public static final int TURRET_LIMIT_SWITCH  = 2;

    public static final double kTurretMinAngle = -90;
    public static final double kTurretMaxAngle = 90;
        //0 is centered

    public static final double kP_Turret = 0.01;
    public static final double kI_Turret = 0.0000;
    public static final double kD_Turret = 0.000;

    public static final double kP_TurretLock = 0.00115;
    public static final double kI_TurretLock = 0.0005;//try .0006
    public static final double kD_TurretLock = 0.000055;//try 0.000007

    public static final double kTurretAngleTolerance = 2.0;

    public static final double kTurretGearReduction = .5;
    public static final double kTurretSprocketRatio = 16.0/120;
    public static final double kTurretDiskDia = 13.75;

    public static final double kTurretSpeedManual = 0.10;//positive is counterclockwise
    public static final double kTurretSpeedAuto = 1.0;
    public static final double kTurretSpeedLock = 0.0625;

  }

  /*
   * Values used for control inputs
   */
  public static final class ControlConstants {

    /**
     * Gamepad Constants
     */

    // Gamepad port IDs
    public static final int XBOX_PORT = 0;

    // Gamepad button IDS
    public static final int xboxAButton = 1;
    public static final int xboxBButton = 2;
    public static final int xboxXButton = 3;
    public static final int xboxYButton = 4;
    public static final int xboxLeftBumper = 5;
    public static final int xboxRightBumper = 6;
    public static final int xboxBackButton = 7;// this button is in the middle of the xbox controller
    public static final int xboxStartButton = 8;// this button is in the middle of the xbox controller
    public static final int xboxLeftJoystickButton = 9;
    public static final int xboxRightJoystickButton = 10;

    public static final double triggerThreshold = 0.3;

    /**
     * Operator Interface Constants
     */

    // OI Button IDs
    public static final int LaunchPadButton1 = 7;
    public static final int LaunchPadButton2 = 17;
    public static final int LaunchPadButton3 = 19;
    public static final int LaunchPadButton4 = 18;
    public static final int LaunchPadSwitch1bottom = 1;
    public static final int LaunchPadSwitch1top = 2;
    public static final int LaunchPadSwitch2bottom = 3;
    public static final int LaunchPadSwitch2top = 4;
    public static final int LaunchPadSwitch3 = 5;
    public static final int LaunchPadSwitch4 = 6;
    public static final int LaunchPadSwitch5bottom = 8;
    public static final int LaunchPadSwitch5top = 9;
    public static final int LaunchPadSwitch6bottom = 10;
    public static final int LaunchPadSwitch6top = 11;
    public static final int LaunchPadSwitch7 = 12;
    public static final int LaunchPadSwitch8 = 13;
    public static final int LaunchPadDial1 = 14; // low bit
    public static final int LaunchPadDial2 = 15;
    public static final int LaunchPadDial3 = 16; // high bit

    /*
     * General Control Constants
     */
    public static final double kJoystickSpeedCorr = 1;
    public static final double kJoystickTolerance = 0.01;

  }

  /**
   * General Robot Constants
   */
  public static final class GeneralConstants {

    // General variables
    public static final double degreesToRads = 0.0174533;
    public static final String CANBUS_NAME = "rio";

    // Filtering (for gyro)
    public static final int FILTER_WINDOW_SIZE = 5;

  }

  /**
   * Mutable variables
   */
  public static final class Mutables {
    
    public static boolean isParked = false;
    public static boolean isFieldOriented = true;
    public static boolean isSlowMode = false;
    public static String autoPosition = "Left";
    public static boolean blueAlliance = true; // true = blue, red = false
    public static boolean photoSensorIsNotBlocked;
    public static boolean impactDetected = false;
    public static boolean isClawClear = false;

  }

}