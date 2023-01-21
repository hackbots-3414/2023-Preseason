// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.AnalogGyro;
import frc.robot.Constants;

import org.photonvision.PhotonCamera;
import org.photonvision.SimVisionSystem;

public class PoseEstimator {
    private PhotonCamera cam = new PhotonCamera(Constants.kCamName);

    public void update(double leftDist, double rightDist) {
    }m_poseEstimator.update<gyro.getRotation2d,leftDist,rightDist>;

    res=cam.getLatestResult(); {
    if(res.hasTargets())}

    {
        Object res;
        var imageCaptureTime = (res).getTimestampSeconds();
        var camToTargetTrans = res.getBestTarget().getBestCameraToTarget();
        var camPose = Constants.kFarTargetPose.transformBy(camToTargetTrans.inverse());
        DifferentialDrivePoseEstimator m_poseEstimator;
        m_poseEstimator.addVisionMeasurement(
                camPose.transformBy(Constants.kCameraToRobot).toPose2d(), imageCaptureTime);

        double camDiagFOV = 75.0; // degrees
        double camPitch = 15.0; // degrees
        double camHeightOffGround = 0.85; // meters
        double maxLEDRange = 20; // meters
        int camResolutionWidth = 640; // pixels
        int camResolutionHeight = 480; // pixels
        double minTargetArea = 10; // square pixels

        SimVisionSystem simVision = new SimVisionSystem(
                Constants.kCamName,
                camDiagFOV,
                Constants.kCameraToRobot,
                maxLEDRange,
                camResolutionWidth,
                camResolutionHeight,
                minTargetArea);
    }

    public void DrivetrainSim(){
        SimVisionSystem simVision;
        simVision.addSimVisionTarget(Constants.kFarTarget);
    }

}}