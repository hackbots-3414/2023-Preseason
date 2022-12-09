// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

// import org.slf4j.Logger;
// import org.slf4j.LoggerFactory;

public class DriveTrain extends SubsystemBase {
  
  // private static final Logger LOG = LoggerFactory.getLogger(DriveTrain.class);
  
  private final DifferentialDriveOdometry m_odometry;

  private AHRS my_ahrs = new AHRS();

  private WPI_TalonFX left_front_motor = new WPI_TalonFX(Constants.LF_MOTOR);
  private WPI_TalonFX right_front_motor = new WPI_TalonFX(Constants.RF_MOTOR);
  private WPI_TalonFX left_back_motor = new WPI_TalonFX(Constants.LB_MOTOR);
  private WPI_TalonFX right_back_motor = new WPI_TalonFX(Constants.RB_MOTOR);
  private DifferentialDrive dfDrive = new DifferentialDrive(left_front_motor, right_front_motor);
  DifferentialDriveVoltageConstraint autoVoltageConstraint =
  new DifferentialDriveVoltageConstraint(
      new SimpleMotorFeedforward(Constants.RobotConstants.ksVolts,
                                 Constants.RobotConstants.kvVoltSecondsPerMeter,
                                 Constants.RobotConstants.kaVoltSecondsSquaredPerMeter),
      Constants.RobotConstants.kDriveKinematics,
      10);
      

  /** Creates a new DriveTrain. */
  public DriveTrain() {
  
    left_front_motor.setSafetyEnabled(true);
    left_back_motor.setSafetyEnabled(true);
    right_front_motor.setSafetyEnabled(true);
    right_back_motor.setSafetyEnabled(true);

    left_back_motor.follow(left_front_motor);
    right_back_motor.follow(right_front_motor);

    right_front_motor.setInverted(true);
    right_back_motor.setInverted(true);
    
    m_odometry = new DifferentialDriveOdometry(my_ahrs.getRotation2d());
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    left_front_motor.feed();
    right_back_motor.feed(); 
    // Update the odometry in the periodic block
    m_odometry.update(
    my_ahrs.getRotation2d(), left_front_motor.getSelectedSensorPosition(), right_front_motor.getSelectedSensorPosition());
  }

  public void drive(double xSpeed, double zRotation) {
    dfDrive.arcadeDrive(xSpeed, zRotation);
  }

  public void resetEncoders() {
    left_front_motor.setSelectedSensorPosition(0);
    right_front_motor.setSelectedSensorPosition(0);  }

  public double getDistance() {
    // this will try to slightly negate any bad motor counting that may happen.
    double sum = left_front_motor.getSelectedSensorPosition() + right_front_motor.getSelectedSensorPosition();
    return sum / 2;
  }

  public double off_by_how_much() {
    /*
    A function to get how off the encoders are (which encoder, if any, is farther ahead). Left - right.
    If zero, no difference.
    If positive, then left value is bigger. (turning counter-clockwise)
    If negative, then right value is bigger. (turning counter clockwise)
    */
    return left_front_motor.getSelectedSensorPosition() - right_front_motor.getSelectedSensorPosition();
  }

  public void resetNavX() {
    my_ahrs.reset();
  }

  public double getZ() {
    double angle =  my_ahrs.getYaw();
    return angle;
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    left_front_motor.setVoltage(leftVolts);
    right_front_motor.setVoltage(rightVolts);
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, my_ahrs.getRotation2d());
    }
}