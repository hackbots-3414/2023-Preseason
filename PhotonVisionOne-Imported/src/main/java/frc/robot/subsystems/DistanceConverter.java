// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DistanceConverter extends SubsystemBase {
  /** Creates a new DistanceConverter. */

  private double FEET_TO_INCHES = 1 / 12;
  private double INCHES_TO_FEET = 12;
  private double METERS_TO_FEET = 1 / 3.28084;
  private double FEET_TO_METERS = 3.28084;

  public DistanceConverter() {}

  public double feetToInches(double feet) {
    return feet * FEET_TO_INCHES;
  }

  public double inchesToFeet(double inches) {
    return inches * INCHES_TO_FEET;
  }

  public double feetToMeters(double feet) {
    return feet * FEET_TO_METERS;
  }

  public double metersToFeet(double meters) {
    return meters * METERS_TO_FEET;
  }

  public double inchesToMeters(double inches) {
    return inches * INCHES_TO_FEET * FEET_TO_METERS;
  }

  public double metersToInches(double meters) {
    return meters * METERS_TO_FEET * FEET_TO_INCHES;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
