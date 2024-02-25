// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Conveyor;

public class RollBack extends Command {

public Conveyor m_conveyor;

  public RollBack(Conveyor conveyor) {
    m_conveyor = conveyor;
  }


  @Override
  public void initialize() {

  }


  @Override
  public void execute() {
    m_conveyor.c_runBoth(0.8);

  }


  @Override
  public void end(boolean interrupted) {
       m_conveyor.c_runBoth(0);

  }

 
  @Override
  public boolean isFinished() {
    return false;
  }
}
