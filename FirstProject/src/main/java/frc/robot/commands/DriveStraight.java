// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystem.DriveTrain;


public class DriveStraight extends CommandBase {
  private DriveTrain drvtrain;
  double targetDistance;
  double drvspeed;
  double turnAdjust;

  /** Creates a new DriveStraight. */
  public DriveStraight(DriveTrain driveTrain, double distanceToDrive, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    targetDistance = distanceToDrive;
    drvtrain = driveTrain;
    drvspeed = speed;
    turnAdjust = 0;
    addRequirements(drvtrain);
    if (distanceToDrive * speed < 0) {
      throw new IllegalArgumentException("DriveStright: distanceToDrive and speed should have the same sign.");
    }

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drvtrain.resetEncoders();
    drvtrain.resetNavX();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drvtrain.drive(0 - drvspeed, 0 - turnAdjust);
    // check to make sure that the robot is NOT turning.
    double angle = drvtrain.off_by_how_much();
    if (-1025 >= angle) {
      turnAdjust += 0.03;
      if (turnAdjust > 1) turnAdjust = 1;
      System.out.println("ADJUSTING POSITIVE");
    } else if (1024 <= angle) {
      turnAdjust -= 0.03;
      if (turnAdjust < -1) turnAdjust = -1;
      System.out.println("ADJUSTING NEGATIVE");
    } else {
      turnAdjust = 0;
    }
    System.out.println("Ticks: " + drvtrain.getDistance());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drvtrain.drive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (targetDistance > 0) {
      return drvtrain.getDistance() >= targetDistance;
    } else if (targetDistance < 0) {
      return drvtrain.getDistance() <= targetDistance;
    } else {
      return true;
    }
  }
}
