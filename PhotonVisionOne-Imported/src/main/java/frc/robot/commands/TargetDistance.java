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
public class TargetDistance extends CommandBase {
  private boolean done;
  private Camera photonvision;
  private DriveTrain driveTrain;
  private boolean targetOne;

  public TargetDistance(DriveTrain driveTrain, Camera photonvision) {
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
