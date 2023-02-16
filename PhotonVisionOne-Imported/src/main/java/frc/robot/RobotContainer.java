// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.GenericHID;
// import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AprilTagChaser;
import frc.robot.commands.Balance;
import frc.robot.commands.DefaultTeleop;
import frc.robot.commands.PIDBalance;
import frc.robot.commands.PhotonCamera;
import frc.robot.commands.TargetDistance;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.DriveTrain;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private DriveTrain drive;
  private static RobotContainer m_robotContainer = new RobotContainer();
  private Camera camera = new Camera();
  XboxController controller = new XboxController(0);
  //SendableChooser<Command> m_chooser = new SendableChooser<>();
  // The robot's subsystems and commands are defined here...

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    drive = new DriveTrain();
    drive.resetEncoders();
    drive.resetHeading();
    drive.setDefaultCommand(new DefaultTeleop(drive));
    SmartDashboard.putData(new PIDBalance(drive));
    // Configure the button bindings
    configureButtonBindings();
    
  }
  public DriveTrain getDriveTrain(){
    return drive;
  }
  private void configureButtonBindings() {
    JoystickButton balance = new JoystickButton(controller, XboxController.Button.kB.value);
    balance.onTrue(new Balance(drive));
  }

  public static RobotContainer getInstance() {
    return m_robotContainer;
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  //private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new AprilTagChaser(drive, camera);
  }



  public XboxController getGamePad() {
    return controller;
  }
}
