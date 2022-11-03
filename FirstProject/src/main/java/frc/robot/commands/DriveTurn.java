// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystem.DriveTrain;

public class DriveTurn extends CommandBase {
  private DriveTrain drvtrain;
  double targetAngle;
  double drvspeed;
  /** Creates a new DriveTurn. */
  public DriveTurn(DriveTrain driveTrain, double target, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    targetAngle = target;
    drvtrain = driveTrain;
    drvspeed = speed;
    addRequirements(drvtrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drvtrain.resetNavX();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drvtrain.drive(0, drvspeed); // NOTE: Does this work? Can i put 0 as a speed?
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drvtrain.drive(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return drvtrain.getZ() > targetAngle;
  }
}
