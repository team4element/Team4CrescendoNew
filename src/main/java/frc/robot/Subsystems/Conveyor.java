// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConveyorConstants;
import frc.robot.Constants.ShooterConstants;

public class Conveyor extends SubsystemBase {
  public static TalonFX m_bottomLeader;
  public static TalonFX m_bottomFollower;
  DutyCycleOut bottomControlRequest = new DutyCycleOut(.8);

  public static TalonFX m_topLeader;
  DutyCycleOut topControlRequest = new DutyCycleOut(.8);

  public static enum Direction {
    INTAKE,
    OUTTAKE
  }

  public Conveyor() {
    // Setting Motors

    // TalonFXConfiguration config = new TalonFXConfiguration();

    // config.CurrentLimits.SupplyCurrentLimit = ConveyorConstants.currentLimitAmps;
    // config.CurrentLimits.SupplyCurrentLimitEnable = true;

    m_bottomLeader = new TalonFX(ConveyorConstants.bottomLeaderId);
    m_bottomFollower = new TalonFX(ConveyorConstants.bottomFollowerId);
    m_bottomFollower.setControl(new Follower(ConveyorConstants.bottomLeaderId, false));

    m_topLeader = new TalonFX(ConveyorConstants.topLeaderId);

    m_bottomLeader.setInverted(true);
    m_topLeader.setInverted(false);

    // m_bottomLeader.getConfigurator().apply(config);
    // m_topLeader.getConfigurator().apply(config);
    // m_bottomFollower.getConfigurator().apply(config);
  }

  public void setBottom(double speed) {
    m_bottomLeader.setControl(bottomControlRequest.withOutput(speed));
  }

  public void setTop(double speed) {
    m_topLeader.setControl(topControlRequest.withOutput(speed));
  }


  // Define all commands here
  public Command c_runTop(Direction direction, double speed) {
    speed = Math.abs(speed);
    double modifiedSpeed = direction == Direction.INTAKE ? speed : -speed;
    return startEnd(
        () -> setTop(modifiedSpeed),
        () -> setTop(0.5));
      
  }

  public Command c_runBottom(Direction direction, double speed) {
    speed = Math.abs(speed);
    double modifiedSpeed = direction == Direction.INTAKE ? speed : -speed;

    return startEnd(
        () -> setBottom(modifiedSpeed),
        () -> setBottom(.5));
  }

  public Command c_runBoth(Direction direction, double speed) {
    double modifiedSpeed = direction == Direction.INTAKE ? speed : -speed;
    return startEnd(() -> {
      setBottom(modifiedSpeed);
      setTop(modifiedSpeed);
    }, () -> {
      setBottom(0);
      setTop(0);
    });
  }
}