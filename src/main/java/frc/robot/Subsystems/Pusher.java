// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.PusherConstants;

public class Pusher extends SubsystemBase {

  public enum Direction {
    SHOOT, RESET
  }

  private WPI_VictorSPX m_motorController;
  private AnalogPotentiometer m_pot;
  private PIDController m_pid;
  private Encoder m_encoder;

  public Pusher() {
    m_motorController = new WPI_VictorSPX(PusherConstants.motorId);
    m_pot = new AnalogPotentiometer(PusherConstants.potID, PusherConstants.potMax, 0);
    m_pid = new PIDController(.0025, 0 , 0.000000);
    m_pid.setTolerance(10);
    m_encoder = new Encoder(1, 2);

  }

  public void setMotor(double speed){
    m_motorController.set(ControlMode.PercentOutput, speed);
  };

  public void controllerOff(){
    m_motorController.set(ControlMode.PercentOutput, 0);
  };

  public void controlPower(double power){
    double encoder_value = getDegree();
    if ((encoder_value >= PusherConstants.potLimitHigh && power > 0) 
    || (encoder_value <= PusherConstants.potLimitLow && power < 0)) {
      setMotor(0);

    }else{
      setMotor(power);
    }
  };

  public void controlPID(double setpoint){
    m_motorController.set(MathUtil.clamp(m_pid.calculate(getDegree(), setpoint), -.5, .5));
  };

  public double getDegree()
  {
    return PusherConstants.potMax - m_pot.get();
  }

  public double getValue() 
  {
    return m_encoder.get();
  }

  public void reset()
  {
    m_encoder.reset();
  }

  @Override
  public void periodic(){
   // System.out.println("Potentiometer:" + getDegree());
    System.out.println("encoder:" + getValue());
  }
}