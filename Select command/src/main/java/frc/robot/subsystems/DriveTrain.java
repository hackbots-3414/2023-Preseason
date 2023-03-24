// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {
  private WPI_TalonFX motor1;
  private WPI_TalonFX motor2;
  private WPI_TalonFX motor3;
  private WPI_TalonFX motor4;
  private DifferentialDrive driveTrain;
  /** Creates a new DriveTrain. */
  public DriveTrain() {
    motor1 = new WPI_TalonFX(10);
    motor2 = new WPI_TalonFX(11);
    motor3 = new WPI_TalonFX(13);
    motor4 = new WPI_TalonFX(14);
    motor2.follow(motor1);
    motor4.follow(motor3);
    motor3.setInverted(TalonFXInvertType.CounterClockwise);
    motor4.setInverted(TalonFXInvertType.CounterClockwise);
    motor1.setInverted(TalonFXInvertType.Clockwise);
    motor2.setInverted(TalonFXInvertType.Clockwise);
    driveTrain = new DifferentialDrive(motor1, motor3);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    driveTrain.feed();
    motor2.feed();
    motor4.feed();
  }
  public void drive(double speed, double steer) {
    driveTrain.arcadeDrive(speed, steer);
    
  }
}
