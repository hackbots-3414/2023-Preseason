// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.PrintCommand;
//import edu.wpi.first.wpilibj2.command.SelectCommand;
//import frc.robot.commands.HeidiLoopyCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.commands.DefaultTeleopCommand;
import frc.robot.commands.DriveStraight;

//import java.util.Map;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private static RobotContainer me = new RobotContainer();
  private XboxController controller = new XboxController(0);
  private DriveTrain drvTrain = new DriveTrain();
  private DriveStraight auton_command = new DriveStraight(drvTrain, 1, 0.1);

  private RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    drvTrain.setDefaultCommand(new DefaultTeleopCommand(drvTrain));
    SmartDashboard.putNumber(Constants.AUTON_CMD_NAME, 0);
  }
  
  public static RobotContainer getInstance() {
    return me;
  }

  public DriveTrain getDriveTain() {
    return drvTrain;
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return auton_command;
  }

  public XboxController getGamePad() {
    return controller;
  }
}