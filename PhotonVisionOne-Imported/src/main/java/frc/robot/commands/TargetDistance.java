// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.DriveTrain;

public class TargetDistance extends CommandBase {
  private boolean done;
  private Camera photonvision;
  private DriveTrain driveTrain;

  public TargetDistance(DriveTrain driveTrain, Camera photonvision) {
    this.photonvision = photonvision;
    this.driveTrain = driveTrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  boolean DistanceToTarget = false;

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentDistance = Camera.getDistanceToTarget(Camera.getLatestResult());
    if (currentDistance < 0) {
      driveTrain.tankDrive(0, 0);
    } else if (currentDistance > 2.3) {
      driveTrain.tankDrive(.3, 0);
    } else if (currentDistance < 1.9) {
      driveTrain.tankDrive(-.3, 0);
    } else {
      driveTrain.tankDrive(0, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.tankDrive(0, 0);
    done = true;
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
