// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.extras;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Constants.GeneralConstants;
import frc.robot.Constants.MechanismConstants;
import frc.robot.Constants.Mutables;

/** Add your docs here. */
public class ScoringCalcs {

    /**
     * Calculates Hub distance, and hub angle from overall robot pose.
     * This method returns nothing but sets values of global constants.
     * 
     * @param robotPose the current overall robot pose
     */
    public static void ShootingCalcs(Field2d robotPose) {

        // Calculates hub angle offset for shooting
        Pose2d pose2d = new Pose2d(-1, -1, Rotation2d.kZero);
        pose2d = robotPose.getRobotPose();
        MechanismConstants.currentY = pose2d.getY();
        double hubY = 0;
        double hubX = 0;
        double xDiff = 0;
        double yDiff = 0;
        if (Mutables.blueAlliance) {

            hubX = GeneralConstants.kBlueHub[0];
            hubY = GeneralConstants.kBlueHub[1];
            
        } else {

            hubX = GeneralConstants.kRedHub[0];
            hubY = GeneralConstants.kRedHub[1];

        }
        yDiff = hubY - pose2d.getY();
        xDiff = hubX - pose2d.getX();
        MechanismConstants.hubAngle = Math.toDegrees(Math.atan(yDiff / (xDiff + 1E-6)));
        MechanismConstants.targetGyroAngle = MechanismConstants.hubAngle;
        MechanismConstants.hubDistance = Math.sqrt((xDiff * xDiff) + (yDiff * yDiff));

    }

}
