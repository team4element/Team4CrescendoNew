// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  
  public static TalonFX m_frontIntake; 
  public static TalonFX m_backIntake;

  public static DutyCycleOut m_leaderRequest;
  public static Follower m_followerRequest;

  public Intake(){

    m_frontIntake = new TalonFX(IntakeConstants.m_frontIntakeID);
    m_backIntake = new TalonFX(IntakeConstants.m_backIntakeID);

    m_leaderRequest = new DutyCycleOut(0);
    m_followerRequest = new Follower(IntakeConstants.m_frontIntakeID, true);

  }

  public void IntakeNote (double Speed) {
    m_leaderRequest.Output = Speed;
    m_frontIntake.setControl(m_leaderRequest);
  }

  public void ReverseNote (double Speed) {
    m_leaderRequest.Output = Speed;
    m_frontIntake.setControl(m_leaderRequest);

  }

  public void PauseIntake() {

    m_frontIntake.set(0);
    m_backIntake.set(0);

  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
