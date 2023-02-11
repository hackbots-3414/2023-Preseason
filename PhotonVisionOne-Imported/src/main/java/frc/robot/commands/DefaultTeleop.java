// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;

public class DefaultTeleop extends CommandBase {
  /** Creates a new DefaultTeleop. */
  DriveTrain drvtrain;
  public DefaultTeleop(DriveTrain drvtrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drvtrain = drvtrain;
    addRequirements(drvtrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed = RobotContainer.getInstance().getGamePad().getLeftY();
    double zRotation = RobotContainer.getInstance().getGamePad().getRightX();
    double speed = Constants.RAMP_SPEED;

    if (xSpeed > 0.5) {
      xSpeed = speed;
    } else if (xSpeed < -0.5) {
      xSpeed = 0 - speed;
    } else {
      xSpeed = 0;
    }

    drvtrain.arcadeDrive(0 - xSpeed, 0 - zRotation);
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
