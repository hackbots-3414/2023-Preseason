// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class PoseEstimator extends SubsystemBase{

    private Camera camera = new Camera();

    public Transform3d getPose(int tagid) {
        Transform3d id = camera.getTransform3d(tagid);
        return id;   
    }
}