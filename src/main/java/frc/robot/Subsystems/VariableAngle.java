// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;


import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AngleConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class VariableAngle extends SubsystemBase {
 
  TalonFX m_angleFollower = new TalonFX(AngleConstants.m_followerID);
  TalonFX m_angleLeader = new TalonFX(AngleConstants.m_leaderID);

  ArmFeedforward forwardControl = new ArmFeedforward(
    AngleConstants.Ks, AngleConstants.Kg, AngleConstants.Kv, AngleConstants.Ka);

  double currentAngle;

  public VariableAngle() {

    m_angleFollower.setControl(new Follower(m_angleLeader.getDeviceID(), true));

  }

  public void setBrakeMode() {
    m_angleLeader.setNeutralMode(NeutralModeValue.Brake);
  }

  public void setCoastMode() {
    m_angleLeader.setNeutralMode(NeutralModeValue.Coast);
  }

  public void armOn(DutyCycleOut speed) {
    m_angleLeader.setControl(speed);
  }

  public void setArmPosition(double position) {
   //m_angleLeader.set(ControlMode.Position, position);
  }

  public void resetSensors() {
    
  }

}
