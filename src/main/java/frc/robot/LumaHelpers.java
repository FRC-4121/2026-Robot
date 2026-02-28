// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.Math;
import java.util.HashMap;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.math.util.Units;

/**
 * Define the Luma Helpers class
 * 
 * This class provides a method for retrieving yaw and distance information
 * from April Tags found on the scoring hub.  The methods within this class
 * are static so the class does not need to be instantiated.
 */
public class LumaHelpers {

    /**
     * Blue side hub April Tags (shooting side only)
     */
    private static final int[] blueTags = {18, 21, 24, 25, 26, 27};
    private static final int[] blueMidTags = {18, 21, 26};

    /**
     * Red side hub April Tags (shooting side only)
     */
    private static final int[] redTags = {2, 5, 8, 9, 10, 11};
    private static final int[] redMidTags = {2, 5, 10};

    /**
     * Get target hub info
     * 
     * Determines the directional yaw and target distance to the scoring hub
     * based on any hub April Tags seen by the Luma P1 camera.
     * 
     * @param camera Luma P1 camera object
     * @param bluealliance Flag indicating current alliance
     * 
     * @return Double array with yaw and distance
     */
    public static double[] getHubTargetInfo(PhotonCamera camera, boolean bluealliance, double camheight, double camangle, double targetheight){
        
        // Initialize local variables
        double[] targetInfo = {0, 0};
        double yawSum = 0.0;
        double distanceSum = 0.0;
        int numTagsFound = 0;
        double avgYaw = 0.0;
        double avgDistance = 0.0;

        // Get the correct hub tags based on current alliance
        int[] hubTags = {};
        if (bluealliance) {
            hubTags = blueMidTags;
        } else {
            hubTags = redMidTags;
        }

        // Get the latest results from the camera pipeline
        var results = camera.getAllUnreadResults();

        // Make sure there are current results before proceeding
        if (!results.isEmpty()) {

            // Camera processed a new frame since last
            // Get the last one in the list.
            var result = results.get(results.size() - 1);

            // Proceed if April Tags were found
            if (result.hasTargets()) {

                // Reset tag count
                numTagsFound = 0;

                // At least one AprilTag was seen by the camera
                for (var target : result.getTargets()) {

                    // Get the April Tag ID
                    int targetID = target.getFiducialId();

                    // Determine if it is one of the taret tags
                    boolean targetFound = false;
                    for (int tag : hubTags){
                        if (tag == targetID){
                            targetFound = true;
                            break;
                        }
                    }

                    // If this is a target tag, continue processing
                    if (targetFound) {

                        // Get yaw and add to running sum
                        yawSum += target.getYaw();

                        // Calculate the distance
                        double distance = PhotonUtils.calculateDistanceToTargetMeters(camheight, targetheight, 
                                                                                      Units.degreesToRadians(camangle), 
                                                                                      Units.degreesToRadians(target.getPitch()));

                        // Add to running sum
                        distanceSum += distance;

                        // Increment target count
                        numTagsFound++;

                    }
                }

                // Calculate final results if at least one tag found
                if (numTagsFound > 0) {

                    // Calculate the averages
                    avgYaw = yawSum / numTagsFound;
                    avgDistance = distanceSum / numTagsFound;

                    // Update target info array
                    targetInfo[0] = avgYaw;
                    targetInfo[1] = avgDistance;

                }
            }

        }

        // Return target info
        return targetInfo;  

    }

    /**
     * Get's target distance
     * @return
     */
    public double getDistance(PhotonCamera camera){

        return 0.0;   
    }

}
