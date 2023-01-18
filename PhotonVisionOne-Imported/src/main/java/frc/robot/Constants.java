// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public final static int LF_MOTOR = 10,
                          LB_MOTOR = 11,
                          RF_MOTOR = 13,
                          RB_MOTOR = 14;
  public final static boolean DEBUG = true;

  public static final class RobotConstants {
    //For Powerup robot:

    
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
    public static final double kRamseteZeta = 0.7;
    /*For Venus robot:
    public static final double kTrackwidthMeters = 0.69;
    public static final DifferentialDriveKinematics kDriveKinematics =
      new DifferentialDriveKinematics(kTrackwidthMeters);  
    public static final double ksVolts = 0.58701;
    public static final double kvVoltSecondsPerMeter = 1.2854;
    public static final double kaVoltSecondsSquaredPerMeter = 0.23564;
    public static final double kPDriveVel = 1.8283;
    public static final double kMaxSpeedMetersPerSecond = 6;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1.38;
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;*/
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
}