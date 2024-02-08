// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BottomConveyor extends SubsystemBase {

  // TODO:change the ID to the actual motor ID of the robot
  public static TalonFX m_firstMotor = new TalonFX(0);
  public static TalonFX m_secondMotor = new TalonFX(0);

  public BottomConveyor() {
  }

  public void Spin(double speed) {
    m_firstMotor.set(speed);
    m_secondMotor.set(speed);

  }

  public void Stop(){
    m_firstMotor.set(0);
    m_secondMotor.set(0);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
