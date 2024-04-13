// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConveyorConstants;

public class Conveyor extends SubsystemBase {
  private static TalonFX m_bottomLeader;
  private static TalonFX m_bottomFollower;
  private static DigitalInput m_limitSwitch;
  private static Spark m_addressable_LEDS;
  private static TalonFX m_topLeader;

  private DutyCycleOut m_bottomControlRequest;
  private DutyCycleOut m_topControlRequest;

  Color m_lastColor;

  public static enum Direction {
    INTAKE,
    OUTTAKE
  }

  public static enum Color{
    BLUE,
    PURPLE,
    ORANGE,
  }

  public Conveyor() {
    m_bottomControlRequest = new DutyCycleOut(.8);
    m_topControlRequest = new DutyCycleOut(.8);

    m_limitSwitch = new DigitalInput(ConveyorConstants.limitSwitchID);
    m_addressable_LEDS = new Spark(ConveyorConstants.blinkinID);

    m_bottomLeader = new TalonFX(ConveyorConstants.bottomLeaderId);
    m_bottomFollower = new TalonFX(ConveyorConstants.bottomFollowerId);
    m_bottomFollower.setControl(new Follower(ConveyorConstants.bottomLeaderId, false));

    m_topLeader = new TalonFX(ConveyorConstants.topLeaderId);

    m_bottomLeader.setInverted(true);
    m_topLeader.setInverted(false);

    m_lastColor = Color.PURPLE;
  }

  public void setBottom(double speed) {
    m_bottomLeader.setControl(m_bottomControlRequest.withOutput(speed));
  }

  public void setTop(double speed) {
    m_topLeader.setControl(m_topControlRequest.withOutput(speed));
  }

  public boolean getLimitSwitch(){
    return m_limitSwitch.get();
  }

  public double parseLedColor(Color color){
    double speed;
    switch (color) {
      case BLUE: speed = .81; break;
      case ORANGE: speed = .63; break;
      case PURPLE: speed = .91; break;

      default: speed = .87; break; //blue
    }
    return speed;
  }

  public void setLEDColor(Color color){
    m_lastColor = color;
    m_addressable_LEDS.set(parseLedColor(color));
    System.out.println("REACH" + color);
  }


  @Override
  public void periodic() {
    m_limitSwitch.get();

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

  public Color lastColor(){
    return m_lastColor;
  }
}

