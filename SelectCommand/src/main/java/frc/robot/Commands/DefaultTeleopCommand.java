// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

public class DefaultTeleopCommand extends CommandBase {
  private Drivetrain driveTrain;

  public DefaultTeleopCommand(Drivetrain drvTrain){
  
  driveTrain = drvTrain;
  addRequirements(driveTrain);
  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed = RobotContainer.getInstance().getGamePad().getLeftY();
    double zRotation = RobotContainer.getInstance().getGamePad().getRightX();
    driveTrain.drive(xSpeed, zRotation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
