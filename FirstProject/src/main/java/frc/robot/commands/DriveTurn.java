// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DriveTurn extends CommandBase {
  private DriveTrain drvtrain;
  double targetRotation;
  double drvspeed;
  /** Creates a new DriveStraight. */
  public DriveTurn(DriveTrain drvtrain1, double rotation, double speed) {
    drvtrain = drvtrain1;
    drvspeed = speed;
    targetRotation = rotation;
    addRequirements(drvtrain);

    if (Math.signum(targetRotation) != Math.signum(speed)){
      throw new IllegalArgumentException("targetRotation and speed must have the same sign");
    }

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
    drvtrain.drive(0,drvspeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drvtrain.drive(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (targetRotation > 0) {
      return drvtrain.getRotation() >= targetRotation;
    }
    else if (targetRotation < 0) {
      return drvtrain.getRotation() <= targetRotation;
    }
    else 
      return true;
  }
}
