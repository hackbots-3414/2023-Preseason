// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.LimeLightGamePiece;

public class FindGamePiece extends CommandBase {
  private DriveTrain driveTrain;
  private LimeLight limeLight;
  private LimeLightGamePiece gamePiece;
  /** Creates a new FindGamePiece. */
  public FindGamePiece(DriveTrain driveTrain, LimeLight limeLight) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrain = driveTrain;
    this.limeLight = limeLight;
    addRequirements(driveTrain);
    addRequirements(limeLight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    gamePiece = limeLight.getNearestGamePiece();
    if (gamePiece == null) {
      return;
    }
    if (gamePiece.getX() >= 3) {
      // turn right
      driveTrain.drive(0, 0.1);
    } else if (gamePiece.getX() < -3) {
      //turn left
      driveTrain.drive(0, -0.1);
    } else {
      driveTrain.drive(0, 0);
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.drive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(limeLight.getNearestGamePiece() == null) {
      return true; 
    } 
    return false;
  }
}