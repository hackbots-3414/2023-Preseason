// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.DriveTrain;

/**
 * robot can follow the target (distance in meters) forwards and backwards
 */
public class AprilTagChaser extends CommandBase {
  private boolean done;
  private Camera photonvision;
  private DriveTrain driveTrain;

  public AprilTagChaser(DriveTrain driveTrain, Camera photonvision) {
    this.photonvision = photonvision;
    this.driveTrain = driveTrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    done = false;
  }

  boolean DistanceToTarget = false;

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentAngle = photonvision.getAngleToTargetRounded(4);
    double turnSpeed;
    if (currentAngle > 0) {
      turnSpeed = -0.2;
    } else if (currentAngle < 0) {
      turnSpeed = 0.2;
    } else {
      turnSpeed = 0;
    }

    double currentDistance = photonvision.getDistanceToTarget();
    if (currentDistance < 0) {
      driveTrain.arcadeDrive(0, 0);
    } else if (currentDistance > 2.3) {
      driveTrain.arcadeDrive(.18, turnSpeed);
    } else if (currentDistance < 1.7) {
      driveTrain.arcadeDrive(-.18, turnSpeed);
    } else if (turnSpeed != 0) {
      driveTrain.arcadeDrive(0, turnSpeed);
    } else {
      driveTrain.arcadeDrive(0,0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.arcadeDrive(0, 0);
    done = true;
  }

  @Override
  public boolean isFinished() {
    return done;
  }
}