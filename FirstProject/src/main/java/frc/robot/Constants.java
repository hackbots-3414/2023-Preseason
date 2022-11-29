// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public final static int LF_MOTOR = 13,
                          LB_MOTOR = 14,
                          RF_MOTOR = 10,
                          RB_MOTOR = 11;


  // Pathweaver Constants
  public static final double ksVolts = 0.63458;
  public static final double kvVoltSecondsPerMeter = 2.4941;
  public static final double kaVoltSecondsSquaredPerMeter = 0.3322;
  public static final double kPDriveVel = 3.3673;
  public static final double kTrackwidthMeters = 0.6096;
  public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);
  public static final double kMaxSpeedMetersPerSecond = 6;
  public static final double kMaxAccelerationMetersPerSecondSquared = 1.38;
  public static final double kRamseteB = 2;
  public static final double kRamseteZeta = 0.7;

  // driving constants
  public static final double kTrackWidthMeters = 0.6096;
  public static final double kWheelDiameter = 0.15915; // in meters //0.15965
  public static final double kTicks = 2048;
  public static final double kGearRatio = 12; // 12 : 1
  public static final double kDistancePerTick = kWheelDiameter * Math.PI / kTicks / kGearRatio; // in meters
  public static final double kInchesPerTick = kDistancePerTick * 39.3701; // Converted meters to Inches

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
