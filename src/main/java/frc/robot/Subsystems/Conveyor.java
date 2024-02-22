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
  public static TalonFX m_bottomLeader;
  public static TalonFX m_bottomFollower; 
  DutyCycleOut bottomControlRequest = new DutyCycleOut(0);

  public Conveyor() {
    // Setting Motors
    m_bottomLeader = new TalonFX(ConveyorConstants.bottomLeaderId);
    m_bottomFollower = new TalonFX(ConveyorConstants.bottomFollowerId);

    m_bottomFollower.setControl(new Follower(ConveyorConstants.bottomLeaderId, false));
  }

  public void intakeBottom() {
    m_bottomLeader.setControl(bottomControlRequest.withOutput(0.5));
  }

  public void setBottomConveyor (double speed) {
    bottomControlRequest.Output = speed;
    m_bottomLeader.setControl(bottomControlRequest);
  }

  public void setTopConveyor(double speed) {
  }

  public void runBothAtSameSpeed(double speed) {
    setBottomConveyor(speed);
    setTopConveyor(speed);
  }

  public void StopBottom(){
    bottomControlRequest.Output = 0.0;
    m_bottomLeader.setControl(bottomControlRequest);
  }

  public void StopTop(){
    // m_top.set(0);
  }

  public void StopBoth(){
    bottomControlRequest.Output = 0.0;
    m_bottomLeader.setControl(bottomControlRequest);
  }
}