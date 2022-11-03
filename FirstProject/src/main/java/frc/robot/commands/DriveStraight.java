// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystem.DriveTrain;

public class DriveStraight extends CommandBase {
  private DriveTrain driveTrain;
  private double distance;
  private double speed;

  public DriveStraight(DriveTrain drvTrain, double distance, double speed) {
    driveTrain = drvTrain;
    this.distance = distance;
    this.speed = speed;
    addRequirements(driveTrain);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    driveTrain.resetEncoders();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    driveTrain.drive((direction ? 0.5 : -0.5), 0);
  }
//HI!
  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    if (driveTrain.getAverageSensorPosition() > distance) {
      return true;
    } else {

      return false;
    }
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    driveTrain.drive(0, 0);
  }
}
