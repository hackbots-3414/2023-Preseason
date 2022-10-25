// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  private WPI_TalonFX motorLeftFront;
  private WPI_TalonFX motorLeftRear;
  private WPI_TalonFX motorRightFront;
  private WPI_TalonFX motorRightRear;

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    motorLeftFront = new WPI_TalonFX(20);
    motorLeftRear = new WPI_TalonFX(21);
    motorRightFront = new WPI_TalonFX(22);
    motorRightRear = new WPI_TalonFX(23);

    motorLeftRear.follow(motorLeftFront);
    motorRightRear.follow(motorRightFront);

    motorRightFront.setInverted(true);
    motorRightRear.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
