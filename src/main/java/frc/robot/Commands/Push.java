// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PusherConstants;
import frc.robot.Subsystems.Pusher;

public class Push extends Command {

  Pusher m_pusher = new Pusher();
  private double m_speed;
  private PIDController pusherPID;
  double m_angle; 

  private static final double tolerance = .5;

  public Push(Pusher pusher, double speed, double angle) {
    this.m_pusher = pusher;
    this.m_speed = speed;
    this.m_angle = angle;

    pusherPID = new PIDController(
      PusherConstants.kP, PusherConstants.kI, PusherConstants.kD);

    pusherPID.setTolerance(
      this.m_pusher.setAngleToTicks(tolerance));

    addRequirements(this.m_pusher);
  }

  @Override
  public void initialize() {
    this.m_pusher.controllerOn(0);
  }


  @Override
  public void execute() {

    double m_speed = pusherPID.calculate(this.m_pusher.getEncoderDistance(), this.m_angle);
    this.m_pusher.controllerOn(m_speed);
    // m_pusher.movePusherToAngle(300);
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
