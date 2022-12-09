// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class turnCommand extends CommandBase {
  private Drivetrain drivetrain;
  private double speed;
  private double angle;
  /** Creates a new turnCommand. */
  public turnCommand(Drivetrain drivetrain, double speed, double angle) {
    this.drivetrain = drivetrain;
    this.speed = speed;
    this.angle = angle;
    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }



  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.getAHRSPosition();
  }



  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
System.out.println("TURN is executed; " + speed);
drivetrain.drive(0, speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("TURN is finished.");
    drivetrain.drive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (angle > 0 && drivetrain.getAHRSPosition() < angle){
      System.out.println("TURN is not finished " + drivetrain.getAHRSPosition());
    return false;
  }
  else if (angle < 0 && drivetrain.getAHRSPosition() > angle){
    return false;
  }
  System.out.println("TURN is finished");
  return true;
}
}
