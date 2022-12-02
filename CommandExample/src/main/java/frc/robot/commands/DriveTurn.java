// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveTurn extends CommandBase {
  private Drivetrain drvtrain;
  double targetAngle;
  double drvspeed;
  boolean clockwise = false;
  /** Creates a new DriveStraight. */
  public DriveTurn(Drivetrain drvtrainPeram, double Angle, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    targetAngle = Math.max(-180, Math.min(180, -Angle)); // clamp value to [-180, 180]
    drvtrain = drvtrainPeram;
    drvspeed = -speed;
    addRequirements(drvtrain);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drvtrain.AHRSReset();
    clockwise = targetAngle > 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (clockwise) drvtrain.drive(0, targetAngle);
    else drvtrain.drive(0, -targetAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drvtrain.drive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double rot = drvtrain.getRot();
    return (rot < targetAngle && clockwise) ? (true) : ((rot > targetAngle && !clockwise) ? (true) : (false));
  }
}
