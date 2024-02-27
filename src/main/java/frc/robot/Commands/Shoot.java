// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
 
package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Shooter;

public class Shoot extends Command {
  public Shooter m_shooter;

  public Shoot(Shooter shooter) {

    m_shooter = shooter;

  }

  @Override
  public void initialize() {

  }


  @Override
  public void execute() {
    m_shooter.motorsOn(.5, false);
    //gives rps (rotations per seconds)
    //78/60 for amp

  }


  @Override
  public void end(boolean interrupted) {
    m_shooter.motorsOff();

  }

  @Override
  public boolean isFinished() {
    return false;
  }
}

