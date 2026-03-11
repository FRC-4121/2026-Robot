// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.extras;
import frc.robot.commands.*;
import frc.robot.LumaHelpers;
import frc.robot.subsystems.*;
import java.math.*;

/*
All units are in meters, seconds, and degrees

*/

/** Add your docs here. */
public class Ballistics2026 {

    //Input Constants
    private double launchAngle;
    private double initialY;
    private double targetY;
    private double gravity = 9.81;
    private double slipFactor; //Adjust this with testing
    private double wheelDiameter;
    private double driveRatio;
    
    public Ballistics2026(double angle, double launchH, double finalH, double slip, double wheelD, double ratio) {

        launchAngle = angle;
        initialY = launchH;
        targetY = finalH;
        slipFactor = slip;
        wheelDiameter = wheelD;
        driveRatio = ratio;
        
    }

    //Calculates time that the projectile will be in the air during the launch
    public double calculateTime(double targetDistance){

        double time = Math.sqrt((targetY - initialY - targetDistance * Math.tan(Math.toRadians(launchAngle)))/ (0.5 * gravity));  

        return time;
    }

    //Calculates the launch velocity of the projectile and using it, calulated the wheel speed necessary to launch 
    //the projectile at that desired velocity
    public double calculateLaunchVelcity(double targetDistance){

        double launchVelocity = targetDistance / (Math.cos(Math.toRadians(launchAngle)) * calculateTime(targetDistance));

        double wheelSpeed = launchVelocity / (Math.PI * wheelDiameter * slipFactor * driveRatio);
    
        return wheelSpeed;
    }    
    
}
