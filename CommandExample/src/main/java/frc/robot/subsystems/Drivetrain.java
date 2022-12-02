// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.RobotContainer;
import frc.robot.Constants.RobotConstants;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {

  private AHRS ahrs = new AHRS();

  private WPI_TalonFX motorFL = new WPI_TalonFX(10);
  private WPI_TalonFX motorFR = new WPI_TalonFX(13);
  private WPI_TalonFX motorBL = new WPI_TalonFX(11);
  private WPI_TalonFX motorBR = new WPI_TalonFX(14);
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
    motorBR.feed();
    motorBL.feed();
  }

  public void drive(double speed, double rot) {
    dDrive.arcadeDrive(speed, rot);
  }

  public void resetEncoders() {
    motorFL.setSelectedSensorPosition(0);
    motorFR.setSelectedSensorPosition(0);
  }

  public double getDistance() {
    // this will try to slightly negate any bad motor counting that may happen.
    // returns distance in meters
    return ((motorFL.getSelectedSensorPosition() + motorFR.getSelectedSensorPosition()) *
      RobotConstants.kWheelCircomference) / (RobotConstants.kTicks * RobotConstants.kGearRatio * 2);
  }

  public void AHRSReset() {
    ahrs.reset();
  }

  public double getRot() {
    return ahrs.getYaw();
  }

}
