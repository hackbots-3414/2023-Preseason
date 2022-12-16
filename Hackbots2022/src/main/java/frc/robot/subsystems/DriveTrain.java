// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {
  private WPI_TalonFX motorLeftFront; 
  private WPI_TalonFX motorLeftBack;
  private WPI_TalonFX motorRightFront;
  private WPI_TalonFX motorRightBack;
  private DifferentialDrive diffDrive;
  private AHRS ahrs;

  
  /** Creates a new DriveTrain. */
  public DriveTrain() {
    motorLeftFront = new WPI_TalonFX(10);
    motorLeftBack = new WPI_TalonFX(11);
    motorRightFront = new WPI_TalonFX(13);
    motorRightBack = new WPI_TalonFX(14);
    
    motorRightFront.setInverted(true);
    motorRightBack.setInverted(true);

    motorLeftBack.follow(motorLeftFront);
    motorRightBack.follow(motorRightFront);

    diffDrive = new DifferentialDrive(motorLeftFront,motorRightFront);
    ahrs = new AHRS(Port.kMXP);
  }
  public void resetAHRS(){
    ahrs.reset();
    
  }
  public void resetEncoders(){
    motorLeftBack.setSelectedSensorPosition(0);

  }
  
  public double getDistance(){
    return motorLeftBack.getSelectedSensorPosition();
  }
  public double getYaw(){
    return ahrs.getYaw();
  }
  
  public void drive(double xSpeed, double zRotation) {
    diffDrive.arcadeDrive(xSpeed, zRotation);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
