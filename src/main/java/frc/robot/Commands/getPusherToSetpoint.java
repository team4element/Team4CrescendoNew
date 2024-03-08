// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Pusher;

public class getPusherToSetpoint extends Command {
  /** Creates a new MoveUntil. */

  private Pusher m_pusher;
  private double m_position;
  private double m_tolerance;
  private int    m_resets;


  public getPusherToSetpoint(Pusher pusher) {
    
    m_pusher = pusher;
    m_resets = 0;
    m_tolerance = .2;
    
    
    addRequirements(m_pusher);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    m_position = m_pusher.getEncoderPosition();
    m_pusher.setMotor(.2);

    if(m_position < m_tolerance  && m_position > -m_tolerance)
    {
      m_resets++;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_pusher.controllerOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_resets == 4;
    }
}
