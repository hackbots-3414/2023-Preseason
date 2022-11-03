// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {

  private WPI_TalonFX left_front_motor = new WPI_TalonFX(10);
  private WPI_TalonFX left_back_motor = new WPI_TalonFX(11); 
  private WPI_TalonFX right_front_motor = new WPI_TalonFX(13);
  private WPI_TalonFX right_back_motor = new WPI_TalonFX(14);
  private DifferentialDrive dfDrive = new DifferentialDrive(left_front_motor, right_front_motor);

  /** Creates a new DriveTrain. */
  public DriveTrain() {

    left_front_motor.setSafetyEnabled(true);
    left_back_motor.setSafetyEnabled(true);
    right_front_motor.setSafetyEnabled(true);
    right_back_motor.setSafetyEnabled(true);

    left_back_motor.follow(left_front_motor);
    right_back_motor.follow(right_front_motor);

    right_front_motor.setInverted(true);
    right_back_motor.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void resetEncoders() {
    right_front_motor.setSelectedSensorPosition(0);
    left_front_motor.setSelectedSensorPosition(0);
  }

  public double getAverageSensorPosition() {
   double average = Math.abs((right_front_motor.getSelectedSensorPosition() + left_front_motor.getSelectedSensorPosition())/2);
   return average;
  }

  public void drive(double xSpeed, double zRotation) {
    dfDrive.arcadeDrive(xSpeed, zRotation);
  }
} 