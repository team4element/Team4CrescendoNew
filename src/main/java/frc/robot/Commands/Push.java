// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Pusher;

public class Push extends Command {
 private Pusher m_pusher;
 private double m_speed;
 private double m_goal;
 private double error;
 

  // private static final double tolerance = .5;

  public Push(Pusher pusher, double speed, double position) {
    m_pusher = pusher;
    m_speed = speed;
    m_goal = position;
    //error = m_goal - m_pusher.zeroEncoder();
    
    addRequirements(this.m_pusher);
  }

 public Push(Pusher pusher, double speed) {
   m_pusher = pusher;
   m_speed = speed;


   addRequirements(this.m_pusher);
 }

  @Override
  public void initialize() {
   m_pusher.setMotor(0);
   m_pusher.zeroEncoder(); 
    

   // try {
   //   m_pusher.Encoder().wait(1);
    //} catch (InterruptedException e) {
      // TODO Auto-generated catch block
   //   e.printStackTrace();
   // }
   

  }

  @Override
  public void execute() {
    this.m_pusher.setMotor(m_speed);
    System.out.print("Encoder placement:" + m_pusher.getEncoderPosition());
  
  }

  @Override
  public void end(boolean interrupted) {
    m_pusher.controllerOff();
    
  }

  @Override
  public boolean isFinished() {

   
    return false;

  }
}