// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.util.Color;
//import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Subsystems.ColorSensor;
//import frc.robot.Constants.PusherConstants;
import frc.robot.Subsystems.Pusher;
// import com.revrobotics.ColorMatch;

public class Push extends Command {

  Pusher m_pusher;
  private double m_speed;
  //private PIDController pusherPID;
  // ColorSensor m_colorSensor;
  // public ColorMatch m_matcher;

   public Color kRed;


  //private static final double tolerance = .5;

  public Push(Pusher pusher, double speed /* , ColorSensor sensor, ColorMatch match*/) {
    m_pusher = pusher;
    m_speed = speed;
    // m_angle = position;
    // m_colorSensor = sensor;
    // m_matcher = match;


    addRequirements(this.m_pusher);
  }

  @Override
  public void initialize() {
    this.m_pusher.controllerOn(0);

    
    // m_matcher.addColorMatch(kRed);
  }


  @Override
  public void execute() {

    //double m_speed = pusherPID.calculate(this.m_pusher.getEncoderDistance(), this.m_angle);
    this.m_pusher.controllerOn(m_speed);
    // m_pusher.movePusherToAngle(300);
    // Color detected = m_colorSensor.getColor();

    String color;
    // ColorMatchResult match = new m_colorSensor.matchClosestColor(detected);


  }


  @Override
  public void end(boolean interrupted) {

    m_pusher.controllerOff();

  }


  @Override
  public boolean isFinished() {
    
    // if(match.color == kRed){
    //   return true;
    // } else {
      return false;
    // }
  }
}
