// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Conveyor;
import frc.robot.Subsystems.Conveyor.Color;

public class BlinkLeds extends Command {
  Conveyor m_conveyor;
  Timer m_timer;
  Color m_lastColor;

  public BlinkLeds(Conveyor conveyor, Color lastColor) {
    m_conveyor = conveyor;
    m_lastColor = lastColor;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(m_conveyor.getLimitSwitch()){
      m_conveyor.setLEDColor(Color.ORANGE);
    } 
    else if (m_lastColor == Color.PURPLE || m_lastColor == Color.ORANGE){
      m_conveyor.setLEDColor(Color.BLUE);
    }else{
      m_conveyor.setLEDColor(Color.PURPLE);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
