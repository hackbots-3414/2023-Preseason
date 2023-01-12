// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {
  private WPI_TalonFX rightFront;
  private WPI_TalonFX rightBack;
  private WPI_TalonFX leftFront;
  private WPI_TalonFX leftBack;

  private DifferentialDrive dr;

  /** Creates a new DriveTrain. */
  public DriveTrain() {
    rightFront = new WPI_TalonFX(13);
    rightBack = new WPI_TalonFX(14);
    leftFront = new WPI_TalonFX(10);
    leftBack = new WPI_TalonFX(11);

    rightBack.follow(rightFront);
    leftBack.follow(leftFront);

    rightBack.setInverted(true);
    rightFront.setInverted(true);
    
    dr = new DifferentialDrive(leftFront, rightFront);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    rightBack.feed();
    rightFront.feed();
    leftFront.feed();
    leftBack.feed();
  }

  public void drive(double xSpeed, double zRotation) {
    dr.arcadeDrive(xSpeed, zRotation);
  }
}
