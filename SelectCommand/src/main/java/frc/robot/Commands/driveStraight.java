// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class driveStraight extends CommandBase {
  private Drivetrain drivetrain;
  private double speed;
  private double distance;
  /** Creates a new driveStraight. */
  public driveStraight(Drivetrain drivetrain, double speed, double distance) {
    this.drivetrain = drivetrain;
    this.speed = speed;
    this.distance = distance;
    addRequirements(drivetrain);
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("DRIVE_STRAIGHT is executed");
    drivetrain.drive(speed, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("DRIVE_STRAIGHT is ended");

    drivetrain.drive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (drivetrain.getEncoderPosition() < distance){
      System.out.println("DRIVE_STRAIGHT is not finished " + drivetrain.getEncoderPosition());
      return false;
    }
    System.out.println("DRIVE_STRAIGHT is finished");
    return true;
  }
}
