// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.math.controller.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.MechanismConstants;
import frc.robot.subsystems.Turret;
import frc.robot.commands.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoTurret extends Command {

  private Turret myTurret;
  private double offset;
  private double speed;
  private double m_kP;
  private double m_kI;
  private double m_kD;

  private double upperLimit;
  private double lowerLimit;
  private double currentPosition;

  private PIDController m_myPIDControl;

  /** Creates a new AutoTurret. */
  public AutoTurret(Turret turret) {

    myTurret = turret;
   
    
    
    addRequirements(myTurret);

  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    speed = -0.5;
    m_kP = 1;
    m_kI = 0;
    m_kD = 0;
    upperLimit = 1.7;
    lowerLimit = -1.7;

    m_myPIDControl = new PIDController(m_kP, m_kI, m_kD);
    m_myPIDControl.setTolerance(0.01);

    currentPosition = myTurret.getPosition();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (MechanismConstants.isTurretEnabled) {

      offset = LimelightHelpers.getTX("limelight-turret") / 100;
      double output = m_myPIDControl.calculate(offset, 0);
      currentPosition = myTurret.getPosition();

      if (currentPosition > upperLimit && output < 0) {
        myTurret.runTurret(0);
      } else if (currentPosition < lowerLimit && output > 0) {
        myTurret.runTurret(0);
      } else {
        myTurret.runTurret(-output * speed);
      }

      SmartDashboard.putNumber("PID Output", output);

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
