// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Pusher;

public class Push extends Command {
  Pusher m_pusher;
 private double m_speed;
  double m_angle;

  // private static final double tolerance = .5;

  public Push(Pusher pusher, double speed, double position) {
    m_pusher = pusher;
  //  m_speed = speed;
    m_angle = position;

    // pusherPID = new PIDController(
    // PusherConstants.kP, PusherConstants.kI, PusherConstants.kD);

    // pusherPID.setTolerance(
    // this.m_pusher.setAngleToTicks(tolerance));
    // private static final double tolerance = .5;
      addRequirements(this.m_pusher);
  }

 public Push(Pusher pusher, double speed) {
   m_pusher = pusher;
   m_speed = speed;


   addRequirements(this.m_pusher);
 }

  @Override
  public void initialize() {
    this.m_pusher.controllerOn(0);
    m_pusher.Encoder().setPosition(0);
    // m_pusher.Encoder().wait(10,0);
   

  }

  @Override
  public void execute() {

    this.m_pusher.controllerOn(m_speed);
  
  }

  @Override
  public void end(boolean interrupted) {

    m_pusher.controllerOff();
    //m_pusher.moveUntilBreak(m_speed);
    

  }

  @Override
  public boolean isFinished() {

   
    return false;

  }
}