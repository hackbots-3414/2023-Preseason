// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class juliaiLoopCommand extends CommandBase {
  private int count = 0;

  /** Creates a new juliaiLoopCommand. */
  public juliaiLoopCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    count = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute () {
    for (int i = 5; i >= 0; i --) {
    int j = 0;
    while (j < 30) { 
      System.out.println("count " + count + ", i: " + 1 + ", j: " + 1);
      j ++;
    }
    }
      count ++;
  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
  return (count > 10);
    
  }
}

