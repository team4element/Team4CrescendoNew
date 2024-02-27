// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Conveyor;

public class Top extends Command {
  /** Creates a new Top. */

  Conveyor m_conveyor = new Conveyor();
  public Top(Conveyor conveyor) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_conveyor = conveyor;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_conveyor.c_runTop(Conveyor.Direction.INTAKE, 0.5);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_conveyor.setTop(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
