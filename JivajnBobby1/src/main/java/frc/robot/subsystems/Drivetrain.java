// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  private WPI_TalonFX lf = new WPI_TalonFX(10);
  private WPI_TalonFX lr = new WPI_TalonFX(11);
  private WPI_TalonFX rf = new WPI_TalonFX(13);
  private WPI_TalonFX rr = new WPI_TalonFX(14);
  private DifferentialDrive df = new DifferentialDrive(lf,rf);
  /** Creates a new Drivetrain. */
  public Drivetrain() {
    lf.setSafetyEnabled(true);
    lr.setSafetyEnabled(true);
    rf.setSafetyEnabled(true);
    rr.setSafetyEnabled(true);

    lr.follow(lf);
    rr.follow(rf);

    rf.setInverted(true);
    rr.setInverted(true);
  
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

 public void drive(double xSpeed, double zRotaion) {
   df.arcadeDrive(xSpeed, zRotaion);
  }
  public void arcadeDrive(double throttle, double steering) {
    df.arcadeDrive(throttle, steering);
}

  public void resetEncoders() {
    lf.setSelectedSensorPosition(0);
  }
  public double getEncoderPosition() {
    return lf.getSelectedSensorPosition();
  }
}