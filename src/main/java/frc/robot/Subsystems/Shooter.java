// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  

 private static TalonFX leftMotor = new TalonFX(ShooterConstants.m_leftMotorID);
  private static TalonFX rightMotor = new TalonFX(ShooterConstants.m_rightMotorID);

  public Shooter() {
   motorsOff();

  }

  @Override
  public void periodic() {
 }

 public void motorsOn(double speed) {
    leftMotor.set(speed);
    rightMotor.set(speed);

 }

  public void motorsOff() {
    leftMotor.set(0);
   rightMotor.set(0);

  }
}
