// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {

  TalonFX m_leftLeader = new TalonFX(ClimberConstants.leftID);
  TalonFX m_rightFollower = new TalonFX(ClimberConstants.rightID);
  Slot0Configs m_config = new Slot0Configs();

  PositionVoltage m_request;

  public Climber() {

    m_request = new PositionVoltage(ClimberConstants.setpointUp).withSlot(0);
    m_leftLeader.setInverted(false);
    m_rightFollower.setControl(new Follower(m_leftLeader.getDeviceID(), true));

    m_rightFollower.setNeutralMode(NeutralModeValue.Brake);
    m_leftLeader.setNeutralMode(NeutralModeValue.Brake);

    SmartDashboard.putNumber(ClimberConstants.tableP, ClimberConstants.kP);
    SmartDashboard.putNumber(ClimberConstants.tableI, ClimberConstants.kI);
    SmartDashboard.putNumber(ClimberConstants.tableD, ClimberConstants.kD);

  }

  public void setMotors(double speed) {

    m_leftLeader.set(speed);
    
 }

 public void resetMotor ()
 {
  m_leftLeader.setPosition(0);
  m_leftLeader.setPosition(0);
 }

  public void motorsOff()
  {
    m_leftLeader.set(0);
  }

  public void setPID()
  {
    double p = SmartDashboard.getNumber(ClimberConstants.tableP, ClimberConstants.kP);
    double i = SmartDashboard.getNumber(ClimberConstants.tableI, ClimberConstants.kI);
    double d = SmartDashboard.getNumber(ClimberConstants.tableD, ClimberConstants.kD);

    m_config.kP = p;
    m_config.kI = i;
    m_config.kD = d;

    m_leftLeader.getConfigurator().apply(m_config);
  }

  public void goToSetPoint(double targetSetpoint){
    m_leftLeader.setControl(m_request.withPosition(targetSetpoint));
  }

  public double getCurrentPosition() {
    // TODO: Fix this?
    return m_leftLeader.getPosition().getAsDouble();
  }

  @Override
  public void periodic() {
    setPID();
    System.out.println("left motor: " + m_leftLeader.getPosition());
    System.out.println("right motor: " + m_rightFollower.getPosition());
  }
}
