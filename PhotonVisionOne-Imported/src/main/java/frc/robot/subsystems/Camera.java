// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Camera extends SubsystemBase {
  private static PhotonCamera camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");
  /** Creates a new ExampleSubsystem. */
  public Camera() {
    
  }
  public static PhotonPipelineResult getLatestResult(){
    return camera.getLatestResult();
  }

  public static PhotonTrackedTarget getBestTarget() {
    return camera.getLatestResult().getBestTarget();
  }

  public static int getID(PhotonTrackedTarget target) { // with a specific target
    return target.getFiducialId();
  }

  public static int getID() { // automatically pick target
    return camera.getLatestResult().getBestTarget().getFiducialId();
  }


  @Override
  public void periodic() {
    super.periodic();
    // System.out.println("Distance is currently: " + getDistanceToTarget());
  }
  public double getDistanceToTarget() {
    return 2;
  }
  /**
   * Finds distance to best target. PLEASE check to make sure that the result is NOT negative. Please ensure that the camera is calibrated, and in 3D mode.
   * @return Distance to target, if target is present. Otherwise, returns -1 for failure.
   */
  public static double getDistanceToTarget(PhotonPipelineResult result) {

    if (result.hasTargets() == false) {
      return -1; // failure
    }

    PhotonTrackedTarget target = result.getBestTarget();
    Transform3d pose = target.getBestCameraToTarget();
    Translation3d targetTranslation = pose.getTranslation();
    Translation3d camera = new Translation3d();
    // initialize a Translation3d with X, Y, and Z values of 0. The camera never will move away from where it is.
    double distance = camera.getDistance(targetTranslation);
    return distance;
  }
}
