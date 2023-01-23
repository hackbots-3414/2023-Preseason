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
  private static PhotonCamera camera = new PhotonCamera("Front_Camera");
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
  /**
   * A private function for use in the Camera subsystem. Returns the robot's position relative to the April Tag.
   * @param result the PhotonPipelineResult that should be used.
   * @return A translation3d object that has the translations for the robot.
   */
  private Translation3d getTranslation3d(PhotonTrackedTarget result) {
    Transform3d pose = result.getBestCameraToTarget();
    Translation3d targetTranslation = pose.getTranslation();
    return targetTranslation;
  }
  /**
   * Finds distance to best target. PLEASE check to make sure that the result is NOT negative. Please ensure that the camera is calibrated, and in 3D mode.
   * @return Distance to target, if target is present. Otherwise, returns -1 for failure.
   */
  public double getDistanceToTarget() {

    PhotonPipelineResult result = camera.getLatestResult();

    if (result.hasTargets() == false) {
      return -1; // failure
    }

    PhotonTrackedTarget target = result.getBestTarget();
    Translation3d targetTranslation = getTranslation3d(target);
    Translation3d camera = new Translation3d();
    // initialize a Translation3d with X, Y, and Z values of 0. The camera never will move away from where it is.
    double distance = camera.getDistance(targetTranslation);
    return distance;
  }
  /**
   * Find the robot's angle relative to the April Tag.
   * @return the angle to the April Tag.
   */
  public double getAngleToTarget() {
    PhotonPipelineResult result = camera.getLatestResult();

    if (result.hasTargets() == false) {
      return 360; // failure
    }

    double angle = result.getBestTarget().getYaw();

    System.out.println("angle=" + angle);

    return angle;
  }
  /**
   * Finds the angle to the April Tag, then returns zero if that angle is within a specified amount. Otherwise, returns the normal angle.
   * @param zeroAtOrig the amount of degrees you will let getAngleToTargetRounded be off by. It doesn't matter if it's positive or negative, the method will account for both.
   * @return The angle that the robot is facing relative to the AprilTag, but zero if the angle is within zeroAt degrees.
   */
  public double getAngleToTargetRounded(double zeroAtOrig) {
    double zeroAt = Math.abs(zeroAtOrig);
    double angle = getAngleToTarget();
    if (angle < zeroAt && angle > 0 - zeroAt) {
      return 0;
    } else {
      return angle;
    }
  }

}
