
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.LimeLightGamePiece.GamePiece;

public class LimeLight extends SubsystemBase {
  /** Creates a new LimeLight. */
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

  public LimeLight() {
    super();
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0); // set pipeline for
                                                                                               // camera
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1); // force off
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(1); // driver cam: turns off
                                                                                              // processing
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(0); // Sets the display to side
                                                                                             // by side IF secondary
                                                                                             // camera is present.
    // NetworkTableInstance.getDefault().getTable("limelight").getEntry("crop").setNumber(2);
  }

  private LimeLightGamePiece getNearestSelectedPiece(GamePiece piece) {
    getLimeLighTable().getEntry("pipeline").setNumber(GamePiece.CUBE == piece ? 1 : 2);
    if (getLimeLighTable().getEntry("tv").getDouble(0) == 0) {
      return null;
    }
    double x = getLimeLighTable().getEntry("tx").getDouble(-1);
    double y = getLimeLighTable().getEntry("ty").getDouble(-1);
    double area = getLimeLighTable().getEntry("ta").getDouble(-1);
    LimeLightGamePiece closest = new LimeLightGamePiece(x, y, area, piece);
    return closest;
  }

  public LimeLightGamePiece getNearestGamePiece() {
    LimeLightGamePiece nearestCone = getNearestSelectedPiece(GamePiece.CONE);
    LimeLightGamePiece nearestCube = getNearestSelectedPiece(GamePiece.CUBE);
    if (nearestCone == null && nearestCube == null) {
      return null;
    }
    if (nearestCone == null && nearestCube != null) {
      return nearestCube;
    }
    if (nearestCone != null && nearestCube == null) {
      return nearestCone;
    }
    return nearestCube.getArea() > nearestCone.getArea() ? nearestCube : nearestCone;
  }

  private NetworkTable getLimeLighTable() {
    return NetworkTableInstance.getDefault().getTable("limelight");
  }

  // post to smart dashboard periodically

  /*
   * SmartDashboard.putNumber("LimelightX", x);
   * 
   * SmartDashboard.putNumber("LimelightY", y);
   * 
   * SmartDashboard.putNumber("LimelightArea", area);
   */

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}