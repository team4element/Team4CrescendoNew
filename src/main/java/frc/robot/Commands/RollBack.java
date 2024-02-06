// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.BottomConveyor;
import frc.robot.Subsystems.TopConveyor;

public class RollBack extends Command {
  
  public TopConveyor m_TopConveyor;
  public BottomConveyor m_BottomConveyor;

  public RollBack(TopConveyor topConveyor, BottomConveyor bottomConveyor) {
    m_TopConveyor = topConveyor;
    m_BottomConveyor = bottomConveyor;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_TopConveyor.Spin(-.5);
    m_BottomConveyor.Spin(-.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_TopConveyor.Stop();
    m_BottomConveyor.Stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
