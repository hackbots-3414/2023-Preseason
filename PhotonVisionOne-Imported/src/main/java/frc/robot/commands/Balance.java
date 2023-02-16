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
    double speed1 = Constants.RAMP_SPEED;
    System.out.println("Tilt: " + angle);

    if (angle < 0 - 2) {
      // Drive forwards
      System.out.println("Drive Forwards!");
      drvtrain.arcadeDrive(0 - speed1 * 1/angle, 0);
    } else if (angle > 2) {
      // Drive backwards
      System.out.println("Drive backwards");
      drvtrain.arcadeDrive(speed1 * 1/angle, 0);
      
    }else {
      // We're level
      System.out.println("LEVEL!!! :)");
      drvtrain.setBrakeMode();
        try {
          wait(5000, 0);
        } catch (InterruptedException e) {
          // TODO Auto-generated catch block
          e.printStackTrace();
        }
      done = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    drvtrain.setCoastMode();
    return done;
  }
}
