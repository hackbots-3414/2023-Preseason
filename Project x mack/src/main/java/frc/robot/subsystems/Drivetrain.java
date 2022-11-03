// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  private WPI_TalonFX motorLeftFront;
  private WPI_TalonFX motorLeftRear;
  private WPI_TalonFX motorRightFront;
  private WPI_TalonFX motorRightRear;
  private DifferentialDrive drive;


  /** Creates a new Drivetrain. */
  public Drivetrain() {
    motorLeftFront = new WPI_TalonFX(10);
    motorLeftRear = new WPI_TalonFX(11);
    motorRightFront = new WPI_TalonFX(13);
    motorRightRear = new WPI_TalonFX(14);

    motorRightRear.follow(motorRightFront); 
    motorLeftRear.follow(motorLeftFront);

    motorLeftFront.setInverted(false);
    motorLeftRear.setInverted(false);
    motorRightFront.setInverted(true);
    motorRightRear.setInverted(true);

    drive = new DifferentialDrive(motorLeftFront, motorRightFront);

  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
  }

  public void function(double speed, double rotation) {
    drive.arcadeDrive(speed, rotation);
  } 

} 

