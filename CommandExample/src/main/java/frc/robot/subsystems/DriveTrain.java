// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {
  private WPI_TalonFX lfMotor = new WPI_TalonFX(21);
  private WPI_TalonFX rfMotor = new WPI_TalonFX(22);
  private WPI_TalonFX lrMotor = new WPI_TalonFX(23);
  private WPI_TalonFX rrMotor = new WPI_TalonFX(24);
  private DifferentialDrive dfDrive = new DifferentialDrive(lfMotor,rfMotor);

  /** Creates a new DriveTrain. */
  public DriveTrain() {

    lfMotor.setSafetyEnabled(true);
    lrMotor.setSafetyEnabled(true);
    rfMotor.setSafetyEnabled(true);
    rrMotor.setSafetyEnabled(true);

    lrMotor.follow(lfMotor);
    rrMotor.follow(rfMotor);

    rfMotor.setInverted(true);
    rrMotor.setInverted(true);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void drive(double xSpeed,double zRotation){
    dfDrive.arcadeDrive(xSpeed, zRotation);
  }
}
