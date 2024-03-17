// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Pusher;

public class Push extends Command {
 private Pusher m_pusher;
 private double m_speed;
 private double m_setpoint;
 

  public Push(Pusher pusher, double setpoint, double speed) {
    m_pusher = pusher;
    m_speed = speed;
    m_setpoint = setpoint;
    
    addRequirements(m_pusher);
  }

  @Override
  public void initialize() {
   m_pusher.setMotor(0);
   
  }

  @Override
  public void execute() {
    m_pusher.setToPosition(m_setpoint, 0.5); //TODO: tune tolerance
  }

  @Override
  public void end(boolean interrupted) {
    m_pusher.controllerOff();
  }

  @Override
  public boolean isFinished() {
    return m_pusher.getDegree() == m_setpoint;
  }
}