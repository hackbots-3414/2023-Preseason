// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveStraight extends CommandBase {
  private Drivetrain drvtrain;
  double targetDistance;
  double drvspeed;
  /** Creates a new DriveStraight. */
  public DriveStraight(Drivetrain drvtrain1, double distanceToDrive, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    targetDistance = distanceToDrive;
    drvtrain = drvtrain1;
    drvspeed = -speed;
    addRequirements(drvtrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drvtrain.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drvtrain.drive(drvspeed, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drvtrain.drive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if ( targetDistance == 0 ) return true;
    return Math.abs(drvtrain.getDistance()) >= Math.abs(targetDistance);
  }
}