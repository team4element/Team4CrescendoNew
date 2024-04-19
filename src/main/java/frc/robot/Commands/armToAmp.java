// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Arm;

public class armToAmp extends Command {
  /** Creates a new armToAmp. */
  Arm m_arm = new Arm();
  double setpoint= 0;

  public armToAmp(Arm arm, double target) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_arm = arm;
    setpoint = target;

    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.amp(setpoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.angleOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_arm.getCurrentPosition() - setpoint) <= 0.1;
  }
}
