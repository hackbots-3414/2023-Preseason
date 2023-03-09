// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.links.Link;
import io.github.pseudoresonance.pixy2api.links.SPILink;

public class PixyCam extends SubsystemBase {
  /** Creates a new PixyCam. */
  private Pixy2 pixy = Pixy2.createInstance(new SPILink());

  public PixyCam() {
    pixy.init();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
