// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;
import java.lang.Math;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Climber;

public class climbToSetpoint extends Command {
  Climber m_climber = new Climber();
  double m_setpoint = 0;

  public climbToSetpoint(Climber climber, double target) {
    m_climber = climber;
    m_setpoint = target;
    addRequirements(climber);
  }


  @Override
  public void initialize() {
    m_climber.goToSetPoint(m_setpoint);
  }

  @Override
  public void end(boolean interrupted) {
    m_climber.motorsOff();
  }

  @Override
  // TODO: Fix this
  public boolean isFinished() {
    // TODO: Test what error there is when it is at the setpoint.
    return Math.abs(m_climber.getCurrentPosition() - m_setpoint) <= 0.1;
  }
}
