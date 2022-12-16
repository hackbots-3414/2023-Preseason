// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class Turn extends CommandBase {
private DriveTrain drivetrain;
private double turn;
  /** Creates a new Turn. */
  public Turn(DriveTrain drivetrain, double turn) {
    this.drivetrain = drivetrain;
    this.turn = turn;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.resetAHRS();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.drive(0, 0.3);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double currentDistance = drivetrain.getDistance();
    if(turn >= 0 && currentDistance < turn){
      return false;
    }
    else if (turn < 0 && currentDistance >= turn){
      return false;
    }
    return true;
  }
}
