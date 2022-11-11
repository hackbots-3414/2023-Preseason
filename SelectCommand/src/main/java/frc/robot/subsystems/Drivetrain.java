// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  private WPI_TalonFX motorLeftFront = new WPI_TalonFX(10);
  private WPI_TalonFX motorLeftRear = new WPI_TalonFX(11);
  private WPI_TalonFX motorRightFront = new WPI_TalonFX(13);
  private WPI_TalonFX motorRightRear = new WPI_TalonFX(14);
  private DifferentialDrive dfDrive = new DifferentialDrive(motorLeftFront, motorRightFront);

  //motorLeftFront = new WPI_TalonFX(10);
   // motorLeftRear = new WPI_TalonFX(11);
   // motorRightFront = new WPI_TalonFX(13);
   // motorRightRear = new WPI_TalonFX(14);

  /** Creates a new Drivetrain. */

  public Drivetrain() {

    motorLeftFront.setSafetyEnabled(true);
    motorLeftRear.setSafetyEnabled(true);
    motorRightFront.setSafetyEnabled(true);
    motorRightRear.setSafetyEnabled(true);
    

    motorLeftRear.follow(motorLeftFront);
    motorRightRear.follow(motorRightFront);

    motorRightFront.setInverted(true);
    motorRightRear.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    dfDrive.feed();
    motorLeftRear.feed();
    motorRightRear.feed();
  }
  public void drive(double xSpeed,double zRotation){
    dfDrive.arcadeDrive(xSpeed, zRotation);
  }
  public void resetEncoders(){
  //  motorLeftRear.setSelectedSensorPosition(0);

  }
  public double getEncoderPosition(){
    return 5;
   // return motorLeftRear.getSelectedSensorPosition();
  }
}
