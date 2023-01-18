// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;


import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


/** Add your docs here. */
public class DriveTrain extends SubsystemBase {
    private WPI_TalonFX motorRightFront;
    private WPI_TalonFX motorRightBack;
    private WPI_TalonFX motorLeftFront;
    private WPI_TalonFX motorLeftBack;
    private DifferentialDrive diffDrive;

    public DriveTrain() {
        motorRightFront = new WPI_TalonFX(10);
        motorRightBack = new WPI_TalonFX(11);
        motorLeftFront = new WPI_TalonFX(13);
        motorLeftBack = new WPI_TalonFX(14);

        motorRightFront.setInverted(true);
        motorRightBack.setInverted(true);

        motorLeftBack.follow(motorLeftFront);
        motorRightBack.follow(motorRightFront);

        diffDrive = new DifferentialDrive(motorLeftFront, motorRightFront);
    }
    public void resetEncoders(){
    motorLeftBack.getSelectedSensorPosition(0);
    }

    public double getDistance(){
        return motorLeftBack.getSelectedSensorPosition();
    }
    public void drive(double xSpeed , double zRotation){
        diffDrive.arcadeDrive(xSpeed, zRotation);
    }
    @Override
    public void periodic(){
    }
}


