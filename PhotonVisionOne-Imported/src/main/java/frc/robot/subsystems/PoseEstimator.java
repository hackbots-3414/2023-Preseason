// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.naming.directory.InvalidSearchControlsException;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.DistanceConverter;

/** Add your docs here. */
public class PoseEstimator extends SubsystemBase{

    private Camera camera = new Camera();
    private DistanceConverter dc = new DistanceConverter();

    public Translation3d aprilTagToCenterField(int id) {
        
        double id1x = dc.inchesToMeters(610.77);
        double id1y = dc.inchesToMeters(42.19);
        double id2x = dc.inchesToMeters(610.77);
        double id2y = dc.inchesToMeters(108.19);
        double id3x = dc.inchesToMeters(610.77);
        double id3y = dc.inchesToMeters(174.19);
        double id4x = dc.inchesToMeters(636.96);
        double id4y = dc.inchesToMeters(265.74);
        double id5x = dc.inchesToMeters(14.25);
        double id5y = dc.inchesToMeters(265.74);
        double id6x = dc.inchesToMeters(40.45);
        double id6y = dc.inchesToMeters(174.19);
        double id7x = dc.inchesToMeters(40.45);
        double id7y = dc.inchesToMeters(108.19);
        double id8x = dc.inchesToMeters(40.45);
        double id8y = dc.inchesToMeters(42.19);
        
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
        return null;

    }
}