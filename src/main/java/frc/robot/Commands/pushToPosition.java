// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Pusher;

public class pushToPosition extends Command {
  /** Creates a new pushToPosition. */
    private Pusher m_pusher;

  public pushToPosition(Pusher pusher) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_pusher = pusher;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_pusher.setToPosition(.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
