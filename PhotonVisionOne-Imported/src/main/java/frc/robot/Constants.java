// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.SimVisionTarget;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.DistanceConverter;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public final static double RAMP_SPEED = 0.18;

  public final static int LF_MOTOR = 10,
                          LB_MOTOR = 11,
                          RF_MOTOR = 13,
                          RB_MOTOR = 14;
  public final static boolean DEBUG = true;

  public static final class RobotConstants {
    /*For Powerup robot:

    
    public static final double kTrackwidthMeters = 0.69;
    public static final double kWheelDiameter = 0.1585; // in meters, cm: 15.85, mm: 158.5
    public static final double kTicks = 2048;
    public static final double kGearRatio = 12; // 12 : 1
    public static final double kDistancePerTick = kWheelDiameter * Math.PI / kTicks / kGearRatio; // in meters
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);  
    public static final double ksVolts = 0.25;
    public static final double kvVoltSecondsPerMeter = 1.2898;
    public static final double kaVoltSecondsSquaredPerMeter = 0.10442;
    public static final double kPDriveVel = 1.512;
    public static final double kMaxSpeedMetersPerSecond = 2;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1.38;
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;*/

    //For Venus robot:
    public static final double kTrackWidthMeters = 0.6096;
    public static final double kWheelDiameter = 0.15915; // in meters //0.15965
    public static final double kTicks = 2048;
    public static final double kGearRatio = 12; // 12 : 1
    public static final DifferentialDriveKinematics kDriveKinematics =
      new DifferentialDriveKinematics(kTrackWidthMeters);  
    public static final double kDistancePerTick = kWheelDiameter * Math.PI / kTicks / kGearRatio; // in meters
    public static final double ksVolts = 0.58701;
    public static final double kvVoltSecondsPerMeter = 1.2854;
    public static final double kaVoltSecondsSquaredPerMeter = 0.23564;
    public static final double kPDriveVel = 1.8283;
    public static final double kMaxSpeedMetersPerSecond = 6;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1.38;
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
  }

  public static final class DriveConstants {
    public static final int kLeftMotorFrontPort = 10; // 10
    public static final int kLeftMotorRearPort = 11;
    public static final int kRightMotorFrontPort = 13;
    public static final int kRightMotorRearPort = 14;
    public static final double voltageRampRate = 0.63;
    public static final double kMaxSpeed = 2.5; // 2
    public static final double driveCurrentLimit = 70.0;
    public static final double triggerThresholdTime = .05;

    public static final double driveLowCurrentLimit = 5.0;
  }

  /**
   * Example of an inner class. One can "import static
   * [...].Constants.OIConstants.*" to gain access
   * to the constants contained within without having to preface the names with
   * the class, greatly
   * reducing the amount of text required.
   */
  public static final class OIConstants {
    // Example: the port of the driver's controller
    public static final int kDriverControllerPort = 0;
  }



public String kCamName;
public static Transform3d kCameraToRobot;
public SimVisionTarget kFarTarget;
public Object kFarTargetPose;

public static class Targets {
  private static DistanceConverter dc = new DistanceConverter();

  private static double id1x = dc.inchesToMeters(610.77);
  private static double id1y = dc.inchesToMeters(42.19);
  private static double id1z = dc.inchesToMeters(18.22);

  private static double id2x = dc.inchesToMeters(610.77);
  private static double id2y = dc.inchesToMeters(108.19);
  private static double id2z = dc.inchesToMeters(18.22);

  private static double id3x = dc.inchesToMeters(610.77);
  private static double id3y = dc.inchesToMeters(174.19);
  private static double id3z = dc.inchesToMeters(18.22);

  private static double id4x = dc.inchesToMeters(636.96);
  private static double id4y = dc.inchesToMeters(265.74);
  private static double id4z = dc.inchesToMeters(27.38);

  private static double id5x = dc.inchesToMeters(14.25);
  private static double id5y = dc.inchesToMeters(265.74);
  private static double id5z = dc.inchesToMeters(27.38);

  private static double id6x = dc.inchesToMeters(40.45);
  private static double id6y = dc.inchesToMeters(174.19);
  private static double id6z = dc.inchesToMeters(18.22);
  
  private static double id7x = dc.inchesToMeters(40.45);
  private static double id7y = dc.inchesToMeters(108.19);
  private static double id7z = dc.inchesToMeters(18.22);

  private static double id8x = dc.inchesToMeters(40.45);
  private static double id8y = dc.inchesToMeters(42.19);
  private static double id8z = dc.inchesToMeters(18.22);

  public static Pose3d target1 = new Pose3d(new Translation3d(id1x, id1y, id1z), new Rotation3d(0.0, 0.0, Units.degreesToRadians(180)));
  public static Pose3d target2 = new Pose3d(new Translation3d(id2x, id2y, id2z), new Rotation3d(0.0, 0.0, Units.degreesToRadians(180)));
  public static Pose3d target3 = new Pose3d(new Translation3d(id3x, id3y, id3z), new Rotation3d(0.0, 0.0, Units.degreesToRadians(180)));
  public static Pose3d target4 = new Pose3d(new Translation3d(id4x, id4y, id4z), new Rotation3d(0.0, 0.0, Units.degreesToRadians(180)));
  public static Pose3d target5 = new Pose3d(new Translation3d(id5x, id5y, id5z), new Rotation3d(0.0, 0.0, Units.degreesToRadians(0)));
  public static Pose3d target6 = new Pose3d(new Translation3d(id6x, id6y, id6z), new Rotation3d(0.0, 0.0, Units.degreesToRadians(0)));
  public static Pose3d target7 = new Pose3d(new Translation3d(id7x, id7y, id7z), new Rotation3d(0.0, 0.0, Units.degreesToRadians(0)));
  public static Pose3d target8 = new Pose3d(new Translation3d(id8x, id8y, id8z), new Rotation3d(0.0, 0.0, Units.degreesToRadians(0)));
  
}

}