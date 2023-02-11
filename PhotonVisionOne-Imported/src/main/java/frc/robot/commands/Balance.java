// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.Constants;

public class Balance extends CommandBase {
  DriveTrain drvtrain;
  boolean done;
  /** Creates a new Balance. */
  public Balance(DriveTrain drvtrain) {
    addRequirements(drvtrain);
    this.drvtrain = drvtrain;
    this.done = false;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double angle = drvtrain.getTilt();
    System.out.println("Pitch: " + angle + "\nYaw: " + drvtrain.getHeading() + "\nRoll: " + drvtrain.getRoll());
    if (angle < 0 - 10) {
      // Drive forwards
      System.out.println("Drive Forwards!");
    } else if (angle > 10) {
      // Drive backwards
      System.out.println("Drive backwards");
    } else {
      // We're level
      System.out.println("LEVEL!!! :)");
      done = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
