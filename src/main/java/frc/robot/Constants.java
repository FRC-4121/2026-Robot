/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

//import com.pathplanner.lib.config.PIDConstants;
import com.ctre.phoenix6.CANBus;

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


    // PathPlanner constants
    //public static final PIDConstants translationConstants = new PIDConstants(4.5, 0.0, 0.0);
    //public static final PIDConstants rotationConstants = new PIDConstants(1.5, 0.0, 0.0);

    // Fast / Slow Mode Constants
    public static double slowModeMultiplier = 1;
    public static double slowModeAngularMultiplier = 1;

  }

  /*
   * Values used for mechanisms
   */
  public static final class MechanismConstants {

    // Set general constants
    public static boolean isDisableState = false;

    // Set shooter PID constants
    public static final double kP_Shoot = .009;
    public static final double kI_Shoot = .001;
    public static final double kD_Shoot = 0;
    public static final double kS_Shoot = 0;
    public static final double kV_Shoot = .145;
    public static final double kA_Shoot = 0;
    public static final double kG_Shoot = 0;

    // Set shooter Motion Magic constants
    public static final double kMagicCruise = 200;
    public static final double kMagicAccel = 1000;
    public static final double kMagicJerk = 1500;

    // Set shooter behavior constants
    public static boolean isShooterMode = true;
    public static boolean stopAutoShooter = false;
    public static double targetVelocity = 0;

    // Set hopper/indexer constants
    public static double kFloorSpeed = 0.3;
    public static final double kIndexerSpeed = 1;

    // Set intake constants
    public static final double kIntakeUp = 0;
    public static final double kIntakeDown = 11.24; // Exact value
    public static boolean isIntakeUp = true;
    public static final double kIntakeSpeedFactor = 0.2;
    public static boolean stopAutoIntake = true;

    // Set intake motor PID constants
    public static final double kP_IntakeLift = 0.06;
    public static final double kI_IntakeLift = 0;
    public static final double kD_IntakeLift = 0.005;

    // Set turret constants
    public static final double kTurretMinAngle = -3.239;
    public static final double kTurretMaxAngle = .35;
        //0 is facing to the climber side of the bot parallel with the front and back frame stock

    // Set ballistics constants
    public static final double kTurretCameraHeight = .5207;
    public static final double kTargetHeight = 1.1176;
    public static final double kTurretCameraAngle = 25;
    public static final double kShooterSlipClose = .25;
    public static final double kShooterSlipFar = .33;
    public static final double kShooterDriveRatio = 2;
    public static final double kShooterWheelDiameter = .1016;

    public static boolean isTurretEnabled = true;
    public static boolean shuttleTurretStatus = false;
    public static boolean turretModeSwitch = false;

    public static double targetDistance;

    public static double shootModeVelocity;
    public static double shuttleModeVelocity;

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

    // Filtering (for gyro)
    public static final int FILTER_WINDOW_SIZE = 5;

    // CANBus variables
    public static final CANBus kDriveBus = new CANBus("rio");
    public static final CANBus kMechBus = new CANBus("Mechanisms");

    //Hub coordinates in meters
    public static final double[] kBlueHub = {4.63, 4.03};
    public static final double[] kRedHub = {11.92, 4.03};

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

  }

}