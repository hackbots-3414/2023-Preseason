// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SuperLooper extends CommandBase {
  private int count;
  
  /** Creates a new SuperLooper. */
  public SuperLooper() {
    // Use addRequirements() here to declare subsystem dependencies.
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    super.initialize();
    count = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("current count:  " + ++count);
    for (int i = 0;i < 10;i++) {
      System.out.println("  i=" + i);
    }
    // Increment count, so that we're not ALWAYS running. That would be quite a boring infinite loop.
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return count == 10;
  }
}
