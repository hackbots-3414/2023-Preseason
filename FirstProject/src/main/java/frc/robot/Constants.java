// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public final static String AUTON_CMD_NAME = "Autonamous Command Option";
  public final static int LF_MOTOR = 10,
  LB_MOTOR = 11, 
  RF_MOTOR = 13, 
  RB_MOTOR = 14;
  /**
   * Example of an inner class. One can "import static [...].Constants.OIConstants.*" to gain access
   * to the constants contained within without having to preface the names with the class, greatly
   * reducing the amount of text required.
   */
  public static final class OIConstants {
    // Example: the port of the driver's controller
    public static final int kDriverControllerPort = 0;
  }
}
