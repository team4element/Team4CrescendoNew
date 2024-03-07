// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Pusher;

public class Push extends Command {
<<<<<<< Updated upstream
  private Pusher m_pusher;
  private double m_speed;
  private double m_angle;
=======
  Pusher m_pusher;
 private double m_speed;
  double m_angle;
 
>>>>>>> Stashed changes

  // private static final double tolerance = .5;

  public Push(Pusher pusher, double speed, double position) {
    m_pusher = pusher;
    m_speed = speed;
    m_angle = position;
    
<<<<<<< Updated upstream
    addRequirements(this.m_pusher);
=======
  

    // pusherPID = new PIDController(
    // PusherConstants.kP, PusherConstants.kI, PusherConstants.kD);

    // pusherPID.setTolerance(
    // this.m_pusher.setAngleToTicks(tolerance));
    // private static final double tolerance = .5;
      addRequirements(this.m_pusher);
>>>>>>> Stashed changes
  }

 public Push(Pusher pusher, double speed) {
   m_pusher = pusher;
   m_speed = speed;


   addRequirements(this.m_pusher);
 }

  @Override
  public void initialize() {
<<<<<<< Updated upstream
    this.m_pusher.setMotor(0);
    m_pusher.zeroEncoder(); 
=======
    this.m_pusher.controllerOn(0);
    m_pusher.Encoder().setPosition( 0);
   // try {
   //   m_pusher.Encoder().wait(1);
    //} catch (InterruptedException e) {
      // TODO Auto-generated catch block
   //   e.printStackTrace();
   // }
   

>>>>>>> Stashed changes
  }

  @Override
  public void execute() {
<<<<<<< Updated upstream
    this.m_pusher.setMotor(m_speed);
=======

    this.m_pusher.controllerOn(m_speed);
    m_pusher.Encoder().notify();
    System.out.println("Encoder Value" + (m_pusher.Encoder().getAbsolutePosition().getValueAsDouble() - 0.402099609375));
>>>>>>> Stashed changes
  
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