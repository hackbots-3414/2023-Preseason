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
  boolean counter_clockwise;
  /** Creates a new DriveTurn. */
  public DriveTurn(DriveTrain drvtrain, double targetAngle, double drvspeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.targetAngle = targetAngle;
    this.drvtrain = drvtrain;
    this.drvspeed = drvspeed;
    addRequirements(this.drvtrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drvtrain.resetNavX();
    if (0 < targetAngle) {
      counter_clockwise = false;
    } else {
      counter_clockwise = true;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (counter_clockwise) {
      drvtrain.drive(0, drvspeed);
    } else {
      drvtrain.drive(0, 0 - drvspeed);
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drvtrain.drive(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double z = drvtrain.getZ();
    System.out.println("Current Angle: " + z);
    if (counter_clockwise) {
      return targetAngle >= z;
    } else {
      return z >= targetAngle;
    }
  }
}
