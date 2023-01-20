// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Camera;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.photonvision.targeting.PhotonPipelineResult;

/** An example command that uses an example subsystem. */
public class PhotonCamera extends CommandBase {
  private boolean done;
  private PhotonPipelineResult result;
  public PhotonCamera(Camera subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    done = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Looking...");
    result = Camera.getLatestResult();
    boolean hasTargets = result.hasTargets();
    if (hasTargets == true) {
      done = true;
      System.out.println("----------------\n\n\n!!!!!!Done is true\n\n\n\n----------------------");
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

public Object getLatestResult() {
    return null;
}
}
