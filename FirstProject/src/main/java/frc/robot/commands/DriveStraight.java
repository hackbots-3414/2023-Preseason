// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DriveStraight extends CommandBase {
  private DriveTrain drvtrain;
  double targetDistance;
  double drvspeed;
  double adjustment;

  /** Creates a new DriveStraight. */
  public DriveStraight(DriveTrain drvtrain1, double distanceToDrive, double speed) {
    if (Math.signum(distanceToDrive) != Math.signum(speed)){
      throw new IllegalArgumentException("DriveStraight: distanceToDrive and speed must have the same sign");
    }
    targetDistance = distanceToDrive;
    drvtrain = drvtrain1;
    drvspeed = speed;
    adjustment = 0;
    addRequirements(drvtrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drvtrain.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drvtrain.drive(-drvspeed,0);
    // double angle = drvtrain.offByHowMuch();
    // if (-1025 >= angle){
    //   adjustment+= 0.03;
    // }
    //   else if (1024<=angle){
    //     adjustment -= 0.03;
    //   if (adjustment > 1) adjustment = 1;
    //   System.out.println("ADJUSTING POSITIVE");
    // } else if (1024 <= angle) {
    //   adjustment -= 0.03;
    //   if (adjustment < -1) adjustment = -1;
    //   System.out.println("ADJUSTING NEGATIVE");
    // } else {
    //   adjustment = 0;
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drvtrain.drive(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if (targetDistance > 0) {
      return drvtrain.getDistance() >= targetDistance;
    }
    else if (targetDistance < 0) {
      return drvtrain.getDistance() <= targetDistance;
    }
    else 
      return true;
        
  }
}
