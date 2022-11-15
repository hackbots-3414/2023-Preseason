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
    this.drvtrain.resetNavX();
    if (0 > targetAngle) {
      this.counter_clockwise = true;
    } else {
      this.counter_clockwise = false;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (this.counter_clockwise) {
      this.drvtrain.drive(0, 0 - this.drvspeed);
    } else {
      this.drvtrain.drive(0, this.drvspeed);
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.drvtrain.drive(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (this.counter_clockwise) {
      return this.targetAngle >= this.drvtrain.getZ();
    } else {
      return this.drvtrain.getZ() >= this.targetAngle;
    }
  }
}
