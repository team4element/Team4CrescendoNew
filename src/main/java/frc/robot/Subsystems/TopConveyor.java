// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TopConveyor extends SubsystemBase {

  // TODO:change the ID to the actual motor ID of the robot
  public static TalonFX m_Motor = new TalonFX(0);

  public TopConveyor() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
