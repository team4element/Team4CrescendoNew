// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {

  TalonFX m_leftLeader = new TalonFX(ClimberConstants.leftID);
  TalonFX m_rightFollower = new TalonFX(ClimberConstants.rightID);


  public Climber() {
 
   // m_leftLeader.setInverted(false);
    m_rightFollower.setControl(new Follower(m_leftLeader.getDeviceID(), true));
  }

  public void flexMotor(double speed) {

      m_leftLeader.set(speed);
  
 }

  public void motorsOff()
  {
    m_leftLeader.set(0);
  }


  public void brake(){

    m_rightFollower.setNeutralMode(NeutralModeValue.Brake);
    m_leftLeader.setNeutralMode(NeutralModeValue.Brake);

  };
}
