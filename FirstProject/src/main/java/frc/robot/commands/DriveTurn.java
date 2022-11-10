// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DriveTurn extends CommandBase {
  private DriveTrain drvtrain;
  double targetDistance;
  double drvspeed;
  double drvrotation;
  /** Creates a new DriveStraight. */
  public DriveTurn(DriveTrain drvtrain1, double rotation, double speed) {
    drvtrain = drvtrain1;
    drvspeed = speed;
    drvrotation = rotation;

    addRequirements(drvtrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drvtrain.resetGyro();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drvtrain.drive(drvspeed,drvrotation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drvtrain.drive(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
        if (targetDistance > 0) {
      return drvtrain.getDistance() >= targetDistance;
    }
    else if (targetDistance < 0) {
      return drvtrain.getDistance() <= targetDistance;
    }
    else 
      return true;
  }
}
