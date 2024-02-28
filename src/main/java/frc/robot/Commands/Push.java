// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Pusher;

public class Push extends Command {

  Pusher m_pusher = new Pusher();

  public Push(Pusher pusher) {
    m_pusher = pusher;
  }

  @Override
  public void initialize() {

  }


  @Override
  public void execute() {

    // m_pusher.controllerOn(0.5);
    m_pusher.movePusherToAngle(300);
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
