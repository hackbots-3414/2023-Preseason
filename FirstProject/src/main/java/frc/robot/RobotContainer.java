// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
/*import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import frc.robot.commands.HeidiLoopyCommand;*/
import frc.robot.subsystems.DriveTrain;
import frc.robot.commands.DefaultTeleopCommand;
import frc.robot.commands.DriveStraight;
import frc.robot.commands.DriveTurn;

//import java.util.Map;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
//  private SequentialCommandGroup commandGroup = new SequentialCommandGroup();
  private static RobotContainer me = new RobotContainer();
  private XboxController controller = new XboxController(0);
  private DriveTrain drvTrain = new DriveTrain();
//  private DriveStraight drive_command = new DriveStraight(drvTrain, -200000, -0.3);
//  private DriveTurn auton_command = new DriveTurn(drvTrain, 90, 0.5);
  private SequentialCommandGroup sequence_command = new SequentialCommandGroup(
                                                    new DriveStraight(drvTrain, 180000, 0.4), 
                                                    new DriveTurn(drvTrain, 70, 0.35),
                                                    new DriveStraight(drvTrain, 550000, 0.4), 
                                                    new DriveStraight(drvTrain, -550000, -0.4), 
                                                    new DriveTurn(drvTrain, -70, -0.35), 
                                                    new DriveStraight(drvTrain, -180000, -0.4));

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
    return sequence_command;
  }

  public XboxController getGamePad() {
    return controller;
  }

}