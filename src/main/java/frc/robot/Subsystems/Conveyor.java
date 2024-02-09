// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConveyorConstants;

public class Conveyor extends SubsystemBase {
  // Define Motors and associated control requests
  // Bottom conveyor
  public static TalonFX m_bottomLeader;
  public static TalonFX m_bottomFollower; 
  DutyCycleOut leaderControlRequest;
  Follower followerControlRequest;

  // Top Conveyor

    public static TalonFX m_top;

  public Conveyor() {

    m_bottomLeader = new TalonFX(ConveyorConstants.leaderID);
    m_bottomFollower = new TalonFX(ConveyorConstants.followerID);

    leaderControlRequest = new DutyCycleOut(0);
    followerControlRequest = new Follower(ConveyorConstants.leaderID, false);
  }

  // TODO: Rename to bottom
  public void setConveyor (double speed) {
    leaderControlRequest.Output = speed;
    m_bottomLeader.setControl(leaderControlRequest);
  }

  public void setTopConveyor(double speed) {
    m_top.set(speed);
  }

  public void runBothAtSameSpeed(double speed) {
    setConveyor(speed);
    setTopConveyor(speed);
  }

  public void StopBottom(){
    leaderControlRequest.Output = 0.0;
    m_bottomLeader.setControl(leaderControlRequest);
  }

  public void StopTop(){
    m_top.set(0);
  }

  public void StopBoth(){
    leaderControlRequest.Output = 0.0;
    m_bottomLeader.setControl(leaderControlRequest);

    m_top.set(0);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
