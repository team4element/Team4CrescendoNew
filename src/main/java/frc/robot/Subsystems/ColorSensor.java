// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PusherConstants;

public class ColorSensor extends SubsystemBase {
  //Creates a new ColorSensor.
  public final I2C.Port i2c = I2C.Port.kOnboard;
   
   public final ColorSensorV3 m_sensor;

   
  public ColorSensor() {
    m_sensor = new ColorSensorV3(i2c);

    kRed = new Color(PusherConstants.kRed1,PusherConstants.kRed2,PusherConstants.kRed3);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
