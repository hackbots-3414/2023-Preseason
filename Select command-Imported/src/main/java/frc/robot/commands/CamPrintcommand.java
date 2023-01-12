// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class CamPrintcommand extends CommandBase {
  /** Creates a new CamPrintcommand. */
  private int counter;
  public CamPrintcommand() {
    // Use addRequirements() here to declare subsystem dependencies.

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    counter = 0; 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    counter ++;
    System.out.println("counter:" + counter);

    for (int iLoop = 0; iLoop < 5; iLoop ++ ) {
      System.out.println("iLoop; " + iLoop);
    }

    int whileLoop = 100;

    while (whileLoop > 90 ){
      System.out.println("whileLoop; " + whileLoop);
      whileLoop = whileLoop - 2;
    }
    do {
      System.out.println("dowhile    " + whileLoop);
      whileLoop ++ ;
     } while (whileLoop < 95);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (counter == 10) {
      return true;
    }
    return false;
  }
}
