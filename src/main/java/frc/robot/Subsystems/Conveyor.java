// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Conveyor extends SubsystemBase {
  // Define Motors and associated control requests
  // Bottom conveyor
  public static TalonFX m_leader;
  public static TalonFX m_follower;
  DutyCycleOut leaderControlRequest;
  Follower followerControlRequest;

  // TODO: Top Conveyor

  public Conveyor() {
    // TODO: Put in Constants
    int leaderID = 0;
    int followerID = 1;

    // TODO: Specify that this is the bottom
    m_leader = new TalonFX(leaderID);
    m_follower = new TalonFX(followerID);

    leaderControlRequest = new DutyCycleOut(0);
    followerControlRequest = new Follower(leaderID, false);
  }

  // TODO: Rename to bottom
  public void setConveyor (double speed) {
    leaderControlRequest.Output = speed;
    m_leader.setControl(leaderControlRequest);
  }

  // TODO: Fix this
  public void setTopConveyor(double speed) {

  }

  public void runBothAtSameSpeed(double speed) {
    setConveyor(speed);
    setTopConveyor(speed);
  }

  // TODO: Need to have one for each conveyor, then one combined.
  public void Stop(){
    leaderControlRequest.Output = 0.0;
    m_leader.setControl(leaderControlRequest);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
