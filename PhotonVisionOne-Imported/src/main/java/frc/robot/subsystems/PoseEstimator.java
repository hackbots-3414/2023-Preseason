// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class PoseEstimator extends SubsystemBase{

    private Camera camera = new Camera();

    public Translation3d aprilTagToCenterField(int id) {
        DistanceConverter dc = new DistanceConverter();
        switch(id) {
            case 1:
                return new Translation3d(id1x, id1y, 0);
            case 2:
                return new Translation3d(id2x, id2y, 0);
            case 3:
                return new Translation3d(id3x, id3y, 0);
            case 4:
                return new Translation3d(id4x, id4y, 0);
            case 5:
                return new Translation3d(id5x, id5y, 0);
            case 6:
                return new Translation3d(id6x, id6y, 0);
            case 7:
                return new Translation3d(id7x, id7y, 0);
            case 8:
                return new Translation3d(id8x, id8y, 0);
        }

    }
}