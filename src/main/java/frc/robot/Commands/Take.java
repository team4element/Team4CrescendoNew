// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Intake;

public class Take extends Command {

  public Intake m_intake = new Intake();

  public Take(Intake intake) {
    m_intake = intake;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {

    m_intake.IntakeNote(.5);

  }

  @Override
  public void end(boolean interrupted) {

    m_intake.PauseIntake();

  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
