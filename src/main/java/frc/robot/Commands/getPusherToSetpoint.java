// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Pusher;

public class getPusherToSetpoint extends Command {
  /** Creates a new MoveUntil. */

  private Pusher m_pusher;
  private double m_setpoint;

  public getPusherToSetpoint(Pusher pusher, double setpoint) {
    
    m_pusher = pusher;
    m_setpoint = setpoint;
    
    addRequirements(m_pusher);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_pusher.resetEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {  
    m_pusher.controlPID(m_setpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_pusher.controllerOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_pusher.isOnSetpoint();
    }
}
