// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class PoseEstimator extends SubsystemBase {
    Camera camera;
    public PoseEstimator(Camera camera) {
        this.camera = camera;
    }

    private DistanceConverter dc = new DistanceConverter();
    /**
     * aprilTagToField uses the coordinates in inches of the aprilTags relative to the bottom left corner according to the field manual
     * @param id The AprilTag that you want to find
     * @return A translation3d that returns the aprilTag distance from the bottom of the field
     */
    private Translation3d aprilTagToField(int id) {
        Translation3d result;
        
        double id1x = dc.inchesToMeters(610.77);
        double id1y = dc.inchesToMeters(42.19);
        double id1z = dc.inchesToMeters(18.22);

        double id2x = dc.inchesToMeters(610.77);
        double id2y = dc.inchesToMeters(108.19);
        double id2z = dc.inchesToMeters(18.22);

        double id3x = dc.inchesToMeters(610.77);
        double id3y = dc.inchesToMeters(174.19);
        double id3z = dc.inchesToMeters(18.22);

        double id4x = dc.inchesToMeters(636.96);
        double id4y = dc.inchesToMeters(265.74);
        double id4z = dc.inchesToMeters(27.38);

        double id5x = dc.inchesToMeters(14.25);
        double id5y = dc.inchesToMeters(265.74);
        double id5z = dc.inchesToMeters(27.38);

        double id6x = dc.inchesToMeters(40.45);
        double id6y = dc.inchesToMeters(174.19);
        double id6z = dc.inchesToMeters(18.22);
        
        double id7x = dc.inchesToMeters(40.45);
        double id7y = dc.inchesToMeters(108.19);
        double id7z = dc.inchesToMeters(18.22);

        double id8x = dc.inchesToMeters(40.45);
        double id8y = dc.inchesToMeters(42.19);
        double id8z = dc.inchesToMeters(18.22);
        
        switch(id) {
            case 1:
                result = new Translation3d(id1x, id1y, id1z);
                break;
            case 2:
                result = new Translation3d(id2x, id2y, id2z);
                break;
            case 3:
                result = new Translation3d(id3x, id3y, id3z);
                break;
            case 4:
                result = new Translation3d(id4x, id4y, id4z);
                break;
            case 5:
                result = new Translation3d(id5x, id5y, id5z);
                break;
            case 6:
                result = new Translation3d(id6x, id6y, id6z);
                break;
            case 7:
                result = new Translation3d(id7x, id7y, id7z);
                break;
            case 8:
                result = new Translation3d(id8x, id8y, id8z);
                break;
            default:
                result = new Translation3d();
                break;
        }
        return result;
    }
    public Translation3d RobotToField(int id) {
        Translation3d robotToAprilTag = camera.getTranslation3d(id);
        Translation3d aprilTagLocation = aprilTagToField(id);

        System.out.println("robotToAprilTag: " + robotToAprilTag.toString() + "\naprilTagToField: " + aprilTagLocation.toString());
        
        double robotToAprilTagX = robotToAprilTag.getX();
        double robotToAprilTagY = robotToAprilTag.getY();
        double robotToAprilTagZ = robotToAprilTag.getZ();

        double aprilTagToFieldX = aprilTagLocation.getX();
        double aprilTagToFieldY = aprilTagLocation.getY();
        double aprilTagToFieldZ = aprilTagLocation.getZ();
       
        double x = robotToAprilTagX + aprilTagToFieldX;
        double y = robotToAprilTagY + aprilTagToFieldY;
        double z = robotToAprilTagZ + aprilTagToFieldZ;
        
        Translation3d robotToField = new Translation3d(x, y, z);
        return robotToField;
    }
}