// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.kauailabs.navx.frc.AHRS;
//import org.slf4j.Logger;
//import org.slf4j.LoggerFactory;

public class DriveTrain extends SubsystemBase {
//  private static final Logger LOG = LoggerFactory.getLogger(DriveTrain.class);

  private WPI_TalonFX lfMotor = new WPI_TalonFX(Constants.LF_MOTOR);  //lf = left front
  private WPI_TalonFX rfMotor = new WPI_TalonFX(Constants.RF_MOTOR);  //rf = right front
  private WPI_TalonFX lbMotor = new WPI_TalonFX(Constants.LB_MOTOR);  //lb = left back
  private WPI_TalonFX rbMotor = new WPI_TalonFX(Constants.RB_MOTOR);  //rb = right back
  private DifferentialDrive dfDrive = new DifferentialDrive(lfMotor, rfMotor);
  private AHRS ahrs = new AHRS();
  
  
  /** Creates a new DriveTrain. */
  public DriveTrain() {

    lfMotor.setSafetyEnabled(true);
    lbMotor.setSafetyEnabled(true);
    rfMotor.setSafetyEnabled(true);
    rbMotor.setSafetyEnabled(true);

    //We tell the lr and rr motors to follow the front ones.
    lbMotor.follow(lfMotor);
    rbMotor.follow(rfMotor);
  
    //Following Hackbots convention, we invert the right motors.
    rfMotor.setInverted(true);
    //Invert is specific to the motor, so even though we told the rr motor to follow rf, we still need to invert it.
    rbMotor.setInverted(true);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    lbMotor.feed();
    rbMotor.feed();
  }

  public void drive(double xSpeed, double zRotation){
    dfDrive.arcadeDrive(xSpeed, zRotation);
  }

  public double getDistance() {
    double sum =  lfMotor.getSelectedSensorPosition() + rfMotor.getSelectedSensorPosition();
    return sum / 2;
    }

  public void resetEncoders(){
    lfMotor.setSelectedSensorPosition(0);
    rfMotor.setSelectedSensorPosition(0);
  }

  public void resetGyro(){
    ahrs.reset();
//    LOG.trace("resetGyr(): resetGyro{}", getRotation());
  }

  public double getRotation() {
//    LOG.trace("getRotation(): getRotation{}", getRotation());
    return ahrs.getAngleAdjustment();
  }

}
