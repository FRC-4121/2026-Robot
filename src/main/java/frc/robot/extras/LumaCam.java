// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.extras;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;


import frc.robot.Constants.*;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;
import java.util.Optional;


/** LumaCam Class
 * 
 * This class repreesnts a single Luma camera on the FRC 4121
 * competition robot.  This class provides methods for connecting 
 * to a Luma camera and for getting a pose estimation based on
 * all April tags within the camera's field of viwe.
 */
public class LumaCam {

    //=== Declare Class Variables ===//
    private PhotonCamera camera;
    private Transform3d robotToCam;
    private PhotonPoseEstimator photonEstimator;
    private Matrix<N3, N1> singleTagStDevs;
    private Matrix<N3, N1> multiTagStDevs;
    private Matrix<N3, N1> curStdDevs;
    private AprilTagFieldLayout tagLayout;
    private CommandSwerveDrivetrain driveTrain;

    /** 
     * Default constructor 
     *
     * @param camName  Name of the camera on the network
     * @param xOffset  X offset of the camera from robot center
     * @param yOffset  Y offset of the camera from robot center
     * @param zOffset  Z offset of the camera from robot center
     * @param roll     The roll of the camera relative to the robor
     * @param pitch    The pitch of the camera relative to the robot
     * @param yaw      The yaw of the camera relative to the robot
     */
    public LumaCam(String camName, double xOffset, double yOffset, double zOffset, double roll, double pitch, double yaw, CommandSwerveDrivetrain drive){

        // Store the drivetrain reference
        driveTrain = drive;

        // Create a camera
        camera = new PhotonCamera(camName);

        // Create a transform to relate camera position
        // to center of the robot
        robotToCam = new Transform3d(new Translation3d(xOffset, yOffset, zOffset),
             new Rotation3d(roll, pitch, yaw));

        // Load AprilTag layout
        tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

        // Create a pose estimator
        photonEstimator = new PhotonPoseEstimator(tagLayout, robotToCam);



    }

    /**
     * Estimates the robot's pose based on idemtified AprilTags
     * 
     */
    public void UpdatePose() {

        // Initialize the pose estimate
        Optional<EstimatedRobotPose> visionEst = Optional.empty();

        // Loop over latest results from camera pipelint and process
        for (var result : camera.getAllUnreadResults()){

            SmartDashboard.putBoolean("Pose Fallback", false);
            visionEst = photonEstimator.estimateCoprocMultiTagPose(result);

            if (visionEst.isEmpty()){

                SmartDashboard.putBoolean("Pose Fallback", true);
                visionEst = photonEstimator.estimateLowestAmbiguityPose(result);
            }

            // Update the accuracy estimates
            UpdateEstimationStdDevs(visionEst, result.getTargets());

            // Add the pose estimation to the drivetrain
            visionEst.ifPresent(
                est -> {

                    // Change our trust in the estimate based on the tags we see
                    var estStdDevs = getEstimationStdDevs();

                    // Create a new Pose2d using the X & Y from the vision estimate along with the current robot gyro angle
                    double poseX = est.estimatedPose.getX();
                    double poseY = est.estimatedPose.getY();
                    double gyro = driveTrain.getCurrentGyro();
                    Pose2d newPose = new Pose2d(poseX, poseY, new Rotation2d(Math.toRadians(gyro)));

                    // Add the pose to the drivetrain
                    driveTrain.addVisionMeasurement(newPose, est.timestampSeconds, estStdDevs);
                }
            );
            
        }

    }

    /**
     * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard
     * deviations based on number of tags, estimation strategy, and distance from the tags.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     * @param targets All targets in this camera frame
     */
    private void UpdateEstimationStdDevs (Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {

        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            curStdDevs = VisionConstants.kSingleTagStDevs;

        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = VisionConstants.kSingleTagStDevs;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an average-distance metric
            for (var tgt : targets) {

                var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty()) continue;
                numTags++;
                avgDist +=
                        tagPose
                                .get()
                                .toPose2d()
                                .getTranslation()
                                .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());

            }

            if (numTags == 0) {

                // No tags visible. Default to single-tag std devs
                curStdDevs = VisionConstants.kSingleTagStDevs;

            } else {

                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                if (numTags > 1) estStdDevs = VisionConstants.kMultiTagStDevs;
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                curStdDevs = estStdDevs;

            }
        }
    }

    /**
     * Returns the latest standard deviations of the estimated pose from {@link
     * #getEstimatedGlobalPose()}, for use with {@link
     * org.wpilib.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}. This should only
     * be used when there are targets visible.
     */
    public Matrix<N3, N1> getEstimationStdDevs() {
        return curStdDevs;
    }

    /**
     * Interface to the consumer of the pose estimation
     */
    @FunctionalInterface
    public interface EstimateConsumer {
        public void accept(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStedDevs);
    }

}
