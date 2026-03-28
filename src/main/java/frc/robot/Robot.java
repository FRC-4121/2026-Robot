// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.MechanismConstants;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import frc.robot.LumaHelpers;
import java.util.Optional;


public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    private PhotonCamera frontCamera;
    public static final Transform3d kRobotToCam = new Transform3d(new Translation3d(0.381, 0.0, 0.0), new Rotation3d(0, 0, 0));

    Field2d fieldPose = new Field2d();

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    public Robot() {
        m_robotContainer = new RobotContainer();
        
        CameraServer.startAutomaticCapture();

        frontCamera = new PhotonCamera("hoppercam");

    }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run();
        m_robotContainer.UpdateStatus();

        //Call pose estimation method
        Optional<EstimatedRobotPose> robotPose = LumaHelpers.getPose(frontCamera, kRobotToCam);
        if (robotPose.isPresent()) {
            EstimatedRobotPose est = robotPose.get();
            Pose2d robotPose2d = est.estimatedPose.toPose2d();
            fieldPose.setRobotPose(robotPose2d);
            SmartDashboard.putData("RobotPose", fieldPose);
        }
        
        

    

    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {

        // Run command to set field centric orientation
        CommandScheduler.getInstance().schedule(m_robotContainer.setFieldCentricGyro());

        // Get the chosen autonomous command
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        // Schedule the auto command to run
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
        
        // Get alliance color from driver station
        m_robotContainer.getAlliance();

    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {

        // Run command to set field centric orientation
        //CommandScheduler.getInstance().schedule(m_robotContainer.setFieldCentricGyro());

        // Stop the autonomous command if it is still running
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }

    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {}
}
