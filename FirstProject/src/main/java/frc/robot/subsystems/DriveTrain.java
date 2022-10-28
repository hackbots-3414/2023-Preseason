// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {
  private WPI_TalonFX lfMotor = new WPI_TalonFX(Constants.LF_MOTOR);  //lf = left front
  private WPI_TalonFX rfMotor = new WPI_TalonFX(Constants.RF_MOTOR);  //rf = right front
  private WPI_TalonFX lrMotor = new WPI_TalonFX(Constants.LB_MOTOR);  //lr = left rear
  private WPI_TalonFX rrMotor = new WPI_TalonFX(Constants.RB_MOTOR);
  private DifferentialDrive dfDrive = new DifferentialDrive(lfMotor, rfMotor);

  /** Creates a new DriveTrain. */
  public DriveTrain() {

    lfMotor.setSafetyEnabled(true);
    lrMotor.setSafetyEnabled(true);
    rfMotor.setSafetyEnabled(true);
    rrMotor.setSafetyEnabled(true);

    //We tell the lr and rr motors to follow the front ones.
    lrMotor.follow(lfMotor);
    rrMotor.follow(rfMotor);
  
    //Following Hackbots convention, we invert the right motors.
    rfMotor.setInverted(true);
    //Invert is specific to the motor, so even though we told the rr motor to follow rf, we still need to invert it.
    rrMotor.setInverted(true);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void drive(double xSpeed, double zRotation){
    dfDrive.arcadeDrive(xSpeed, zRotation);
  }

  public void resetEncoders(){
    lfMotor.setSelectedSensorPosition(0);
    rfMotor.setSelectedSensorPosition(0);
  }

  public double getDistance() {
    double sum =  lfMotor.getActiveTrajectoryPosition() + rfMotor.getActiveTrajectoryPosition();
    return sum / 2;
    }
}
