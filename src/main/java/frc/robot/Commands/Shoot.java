// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Shooter;

public class Shoot extends Command {
  private Shooter m_shooter;
  private double m_rpm;

  public Shoot(Shooter shooter, double rpm) {

    m_shooter = shooter;
    m_rpm = rpm;

  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    m_shooter.setMotorRPM(rpm_to_rps(m_rpm), true);
    // m_shooter.setMotorRPM(.5, false);

    rpm_to_rps(0); //this is here to silence a warning
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
