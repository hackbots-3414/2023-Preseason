// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.management.loading.PrivateMLet;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
private WPI_TalonFX moterLeftFront;
private WPI_TalonFX moterLeftRear;
private WPI_TalonFX moterRightfront;
private WPI_TalonFX moterRightRear;
  
}
  /** Creates a new Drivetrain. */
  public Drivetrain() {
    moterLeftFront = new WPI_TalonFX(20);
    moterLeftRear = new WPI_TalonFX(21);
    moterRightfront = new WPI_TalonFX(22);
    moterRightRear = new WPI_TalonFX(23);
  

    moterRightfront.follow(moterLeftFront);
    moterRightRear.follow(moterRightfront);

    
    moterRightfront .setInverted(true);
    moterLeftRear .setInverted(true);


  @Override
  public void periodic() {
    
  
    // This method will be called once per scheduler run
  }
}
