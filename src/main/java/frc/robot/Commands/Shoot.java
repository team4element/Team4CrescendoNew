// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Shooter;

public class Shoot extends Command {
  private Shooter m_shooter;
  private double m_rpmTop;
  private double m_rpmBot;

  public Shoot(Shooter shooter, double rpmTop, double rpmBottom) {
    m_shooter = shooter;
    m_rpmTop = rpmTop;
    m_rpmBot = rpmBottom;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    m_shooter.setMotorRPM(rpm_to_rps(m_rpmTop), rpm_to_rps(m_rpmBot));
  }

  @Override
  public void end(boolean interrupted) {
    m_shooter.motorsOff();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  private double rpm_to_rps(double rpm) {
    final int mins_to_secs = 60;

    return rpm / mins_to_secs;
  }

}
