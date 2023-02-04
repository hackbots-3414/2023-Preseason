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
}