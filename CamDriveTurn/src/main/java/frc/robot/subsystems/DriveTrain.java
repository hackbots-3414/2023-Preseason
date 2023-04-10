// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class DriveTrain extends SubsystemBase {
  private WPI_TalonFX rightFront = new WPI_TalonFX(1);
  private WPI_TalonFX leftFront = new WPI_TalonFX(2);
  private WPI_TalonFX rightBack = new WPI_TalonFX(3);
  private WPI_TalonFX leftBack = new WPI_TalonFX(4);
  private AHRS ahrs = new AHRS(Port.kMXP);
  
  private final DifferentialDriveOdometry rotation;

  private DifferentialDrive drive;
  /** Creates a new DriveTrain. */
  public DriveTrain() {
    rightBack.follow(rightFront);
    leftBack.follow(leftFront);
    rightFront.setInverted(TalonFXInvertType.Clockwise);
    rightBack.setInverted(TalonFXInvertType.Clockwise);
    leftFront.setInverted(TalonFXInvertType.CounterClockwise);
    leftBack.setInverted(TalonFXInvertType.CounterClockwise);

    drive = new DifferentialDrive(rightFront, leftFront);

    rotation = new DifferentialDriveOdometry(ahrs.getRotation2d(), (leftFront.getSelectedSensorPosition() + leftBack.getSelectedSensorPosition()), (rightFront.getSelectedSensorPosition() + rightBack.getSelectedSensorPosition()));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    rightFront.feed();
    leftFront.feed();
    rightBack.feed();
    leftBack.feed();

    rotation.update(ahrs.getRotation2d(), leftFront.getSelectedSensorPosition(), rightFront.getSelectedSensorPosition());
  }
  public void drive(double xSpeed, double zRotation) {

    drive.arcadeDrive(xSpeed, zRotation);  
  }
  public void zeroNavX()
  {
    ahrs.reset();
  }
  public double getHeading()
  {
    return ahrs.getQuaternionY();
  }
}
