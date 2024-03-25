// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Climber;

public class climbToSetpoint extends Command {

  Climber m_climber = new Climber();

  public climbToSetpoint(Climber climber) {
 
    m_climber = climber;
  
  }


  @Override
  public void initialize() {
    m_climber.resetMotor();
  }


  @Override
  public void execute() {
    m_climber.goToSetPoint();
  }

  @Override
  public void end(boolean interrupted) {
    m_climber.motorsOff();
    m_climber.brake();
  }


  @Override
  public boolean isFinished() {
    return false;
  }
}
