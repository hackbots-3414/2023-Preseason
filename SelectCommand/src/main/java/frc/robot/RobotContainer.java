// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.DefaultTeleopCommand;
import frc.robot.Commands.driveStraight;
import frc.robot.Commands.turnCommand;
import frc.robot.subsystems.Drivetrain;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */

public class RobotContainer {
  private static RobotContainer me = new RobotContainer();
  private XboxController gamepad = new XboxController(0);
  private Drivetrain drivetrain = new Drivetrain();



  // The enum used as keys for selecting the command to run.
  private enum CommandSelector {
    ONE,
    TWO,
    THREE,
    CUATRO,
    DRIVE_STRAIGHT
  }

  // An example selector method for the selectcommand.  Returns the selector that will select
  // which command to run.  Can base this choice on logical conditions evaluated at runtime.
  private CommandSelector select() {
    return CommandSelector.DRIVE_STRAIGHT;

  }

  // An example selectcommand.  Will select from the three commands based on the value returned
  // by the selector method at runtime.  Note that selectcommand works on Object(), so the
  // selector does not have to be an enum; it could be any desired type (string, integer,
  // boolean, double...)

  //private final Command m_exampleSelectCommand = new driveStraight(drivetrain, -0.45, -15000);
  //private final Command m_exampleSelectCommand = new turnCommand(drivetrain, -0.3, 90);
  private final Command m_exampleSelectCommand = new SequentialCommandGroup(new driveStraight(drivetrain, 0.4, 175000), 
  new turnCommand(drivetrain, 0, 0), 
  new driveStraight(drivetrain, 0, 0), 
  new turnCommand(drivetrain, 0.3, 90));
 // private final Command m_exampleSelectCommand = new SequentialCommandGroup(new driveStraight(drivetrain, 0, 0), 
  //new turnCommand(drivetrain, 0.3, 90));
     /* new SelectCommand(
          // Maps selector values to commands
          Map.ofEntries(
              Map.entry(CommandSelector.ONE, new PrintCommand("Command one was selected!")),
              Map.entry(CommandSelector.TWO, new PrintCommand("Command two was selected!")),
              Map.entry(CommandSelector.THREE, new PrintCommand("Command three was selected!")),
              Map.entry(CommandSelector.DRIVE_STRAIGHT, new driveStraight(drivetrain, 0.5, 10000))),
          this::select);*/

  private RobotContainer() {
    // Configure the button bindings
    drivetrain.setDefaultCommand(new DefaultTeleopCommand(drivetrain));
    configureButtonBindings();
    System.out.println("in robot container constructor");
    
  }

  public static RobotContainer getInstance(){
    return me;
  }

  public Drivetrain getDrivetrain(){
    return drivetrain;
  }
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_exampleSelectCommand;
  }

  public XboxController getGamePad(){
    return gamepad;
  }
}
