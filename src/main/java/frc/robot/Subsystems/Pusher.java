// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Victor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PusherConstants;

public class Pusher extends SubsystemBase {

  private Victor m_motorController = new Victor(PusherConstants.m_controllerID);

  public Pusher() {
    controllerOff();
  }

  public void controllerOn(double speed){
    m_motorController.set(.5);
  };

  public void controllerOff(){
    m_motorController.set(0);
  };

  @Override
  public void periodic() {
    
  }
}