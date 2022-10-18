// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import java.util.Map;
import frc.robot.commands.SuperLooper;
import frc.robot.subsystem.DriveTrain;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private  static RobotContainer me = new RobotContainer();
  private XboxController controller = new XboxController(0);
  private DriveTrain drvTrain = new DriveTrain();
 

  // The enum used as keys for selecting the command to run.
  private enum CommandSelector {
    ONE,
    TWO,
    THREE,
    FOUR,
    EMPTY,
    INVALID
  }

  // An example selector method for the selectcommand.  Returns the selector that will select
  // which command to run.  Can base this choice on logical conditions evaluated at runtime.
  private CommandSelector select() {
    double selection = SmartDashboard.getNumber(Constants.AUTON_CMD_NAME, 0);
    if (selection == 0.00) {
      return CommandSelector.EMPTY;
    } else if (selection == 1.00) {
      return CommandSelector.ONE;
    } else if (selection == 2.00) {
      return CommandSelector.TWO;
    } else if (selection == 3.00) {
      return CommandSelector.THREE; 
    } else if (selection == 4.00) {
      return CommandSelector.FOUR;
    }
    // anything else is conidered invalid...
    return CommandSelector.INVALID;

  }

  // An example selectcommand.  Will select from the three commands based on the value returned
  // by the selector method at runtime.  Note that selectcommand works on Object(), so the
  // selector does not have to be an enum; it could be any desired type (string, integer,
  // boolean, double...)
  private final Command m_exampleSelectCommand =
      new SelectCommand(
          // Maps selector values to commands
          Map.ofEntries(
              Map.entry(CommandSelector.ONE, new PrintCommand("Command one was selected!")),
              Map.entry(CommandSelector.TWO, new PrintCommand("Command two was selected!")),
              Map.entry(CommandSelector.THREE, new PrintCommand("Command three was selected!")),
              Map.entry(CommandSelector.FOUR, new SuperLooper()),
              Map.entry(CommandSelector.EMPTY, new PrintCommand("No command has been selected yet. Perhaps put tomething there?")),
              Map.entry(CommandSelector.INVALID, new PrintCommand("The option you selected is quite invalid."))),
          this::select);

  private RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    SmartDashboard.putNumber(Constants.AUTON_CMD_NAME, 0);
  }

  public static RobotContainer getInstance() {
    return me;
  }

  public DriveTrain getDriveTain() {
    return drvTrain;
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

  public XboxController getGamePad() {
    return controller;
  }
}