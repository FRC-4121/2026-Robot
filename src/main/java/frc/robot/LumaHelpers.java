// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonCamera;

/** Add your docs here. */
public class LumaHelpers {

    // Create red and blue hub tag lists
    private int[] blueTags = {18, 21, 24, 25, 26, 27};
    private int[] redTags = {2, 5, 8, 9, 10, 11};

    /**
     * Create a LumaHelpers class
     */
    public LumaHelpers() {

    }

    /**
     * Get's target yaw angle
     * @return
     */
    public double getYaw(PhotonCamera camera){
        
        // var results = camera.getAllUnreadResults();
        // if (!results.isEmpty()) {
        //     // Camera processed a new frame since last
        //     // Get the last one in the list.
        //     var result = results.get(results.size() - 1);
        //     if (result.hasTargets()) {
        //         // At least one AprilTag was seen by the camera
        //         for (var target : result.getTargets()) {

        //             int targetID = target.getFiducialId();
        //             boolean 

        //             if (target.getFiducialId() == 7) {
                       
        //                 targetYaw = target.getYaw();
        //                 targetVisible = true;
        //             }
        //         }
        //     }
        return 0;   
    }

    /**
     * Get's target distance
     * @return
     */
    public double getDistance(PhotonCamera camera){

        return 0;   
    }

}
