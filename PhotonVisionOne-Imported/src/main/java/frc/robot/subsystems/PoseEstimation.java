// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.SimVisionSystem;
import frc.robot.Constants;
import frc.robot.commands.PhotonCamera;

/** Add your docs here. */
public class PoseEstimation {
    private PhotonCamera cam = new PhotonCamera(null);
    
    public void update (double frontLeft , double frontRight) {
        PhotonCamera m_PoseEstiPhotonCamera;
        //m_PoseEstiPhotonCamera.update(gyro.getRotation2d(), frontLeft , frontRight);
        var res = cam.getLatestResult();
        
    }
    double camDiagFOV = 75.0; // degrees
    double camPitch = 15.0; // degrees
    double camHeightOffGround = 0.85; // meters
    double maxLEDRange = 20; // meters
    int camResolutionWidth = 640; // pixels
    int camResolutionHeight = 480; // pixels
    double minTargetArea = 10; // square pixels

    SimVisionSystem simVision =
            new SimVisionSystem(
                    Constants.kCamName,
                    camDiagFOV,
                    Constants.kCameraToRobot,
                    maxLEDRange,
                    camResolutionWidth,
                    camResolutionHeight,
                    minTargetArea);
}



