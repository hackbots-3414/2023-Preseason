// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

public class DefaultTeleopCommand extends CommandBase {
  private Drivetrain drvtrain;

  /** Creates a new DefaultTeleopCommand. */
  public DefaultTeleopCommand(Drivetrain drvtrain1) {
    // Use addRequirements() here to declare subsystem dependencies.
    drvtrain = drvtrain1;
    addRequirements(drvtrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drvtrain.drive(RobotContainer.getInstance().getGamepad().getLeftY(), RobotContainer.getInstance().getGamepad().getLeftY());
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
