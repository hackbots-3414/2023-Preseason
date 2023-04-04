// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class DriveTrain extends SubsystemBase {
  private WPI_TalonFX motor1 = new WPI_TalonFX(1);
  private WPI_TalonFX motor2 = new WPI_TalonFX(2);
  private WPI_TalonFX motor3 = new WPI_TalonFX(3);
  private WPI_TalonFX motor4 = new WPI_TalonFX(4);
  private AHRS navX = new AHRS();

  private DifferentialDrive drive;
  /** Creates a new DriveTrain. */
  public DriveTrain() {
    motor3.follow(motor1);
    motor4.follow(motor2);
    motor1.setInverted(TalonFXInvertType.Clockwise);
    motor3.setInverted(TalonFXInvertType.Clockwise);
    motor2.setInverted(TalonFXInvertType.CounterClockwise);
    motor4.setInverted(TalonFXInvertType.CounterClockwise);

    drive = new DifferentialDrive(motor1, motor2);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    motor1.feed();
    motor2.feed();
    motor3.feed();
    motor4.feed();

  }
  public void drive(double xSpeed, double zRotation) {

    drive.arcadeDrive(xSpeed, zRotation);
    if (navX.getQuaternionY() >= 90);
      return;
    if (navX.getQuaternionY() < 90);
      

  }
  public void zeroNavX()
  {
    navX.reset();
  }
  public double getHeading()
  {
    return navX.getQuaternionY();
  }
}
