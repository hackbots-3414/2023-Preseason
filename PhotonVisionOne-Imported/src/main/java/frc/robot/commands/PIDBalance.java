// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

public class PIDBalance extends PIDCommand {
  private DriveTrain drvTrain;
  /** Creates a new PIDBalance. */
  public PIDBalance(DriveTrain drvTrain) {
    super(
        // The controller that the command will use
        new PIDController(0.01, 0, 0),
        // This should return the measurement
        drvTrain::getTilt,
        // This should return the setpoint (can also be a constant)
        () -> 0,
        // This uses the output
        output -> {
          // Use the output here
          if (output > .25){
            output = .25;
          }
          else if (output < -.25){
            output = -.25;
          }
          drvTrain.arcadeDrive(-output, 0);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    this.drvTrain = drvTrain;
    addRequirements(drvTrain);
    SmartDashboard.putData(getController());
    getController().enableContinuousInput(-180, 180);
    getController().setTolerance(2.4, 0.01);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
