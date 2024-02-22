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
  public static TalonFX m_bottomLeader;
  public static TalonFX m_bottomFollower; 
  DutyCycleOut bottomControlRequest = new DutyCycleOut(0);

  public static TalonFX m_topLeader;
  DutyCycleOut topControlRequest = new DutyCycleOut(0);

  public Conveyor() {
    // Setting Motors
    m_bottomLeader = new TalonFX(ConveyorConstants.bottomLeaderId);
    m_bottomFollower = new TalonFX(ConveyorConstants.bottomFollowerId);
    m_bottomFollower.setControl(new Follower(ConveyorConstants.bottomLeaderId, false));

    m_topLeader = new TalonFX(ConveyorConstants.topLeaderId);
  }

  public void setBottom(double speed) {
    m_bottomLeader.setControl(bottomControlRequest.withOutput(speed));
  }

  public void setTop(double speed) {
    m_topLeader.setControl(topControlRequest.withOutput(speed));
  }
}