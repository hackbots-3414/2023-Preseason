// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  private WPI_TalonFX motorFL = new WPI_TalonFX(20);
  private WPI_TalonFX motorFR = new WPI_TalonFX(21);
  private WPI_TalonFX motorBL = new WPI_TalonFX(22);
  private WPI_TalonFX motorBR = new WPI_TalonFX(23);
  private DifferentialDrive dDrive = new DifferentialDrive(motorFL, motorFR);
  /** Creates a new Drivetrain. */
  public Drivetrain() {
    motorFL.setSafetyEnabled(true);
    motorFR.setSafetyEnabled(true);
    motorBL.setSafetyEnabled(true);
    motorBR.setSafetyEnabled(true);

    motorBL.follow(motorFL);
    motorBR.follow(motorFR);

    motorFR.setInverted(true);
    motorBR.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void drive(double speed, double rot) {
    dDrive.arcadeDrive(speed, rot);
  }
}
