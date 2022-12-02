// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

public class DriveStraight extends CommandBase {
  double speed;
  double distance;
  private final double requestedDistance;


  private final Drivetrain m_drivetrain;
  /** Creates a new DriveStraight. */
  public DriveStraight(Drivetrain subsystem, double ipdistance) {
    distance = ipdistance;
    speed = 0.6;
    if (ipdistance < 0) {
      speed = -speed;
    }

    this.requestedDistance = ipdistance;
    

    m_drivetrain = subsystem; 
   
        addRequirements(subsystem);
   
    // Use addRequirements() here to declare subsystem dependencies.
  }
    
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.distance = requestedDistance;
    m_drivetrain.arcadeDrive(0, 0);
    m_drivetrain.resetEncoders();

    this.speed = Math.copySign(this.speed, this.distance);
    this.distance = Math.abs(this.distance / Constants.RobotConstants.kTicksperInch);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.arcadeDrive(speed, 0);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(m_drivetrain.getEncoderPosition()) < Math.abs(this.distance)) {
      return false;
  } else {
      return true;
  }
}
}
