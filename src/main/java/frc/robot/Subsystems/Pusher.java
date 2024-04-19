// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.PusherConstants;

public class Pusher extends SubsystemBase {

  public enum Direction {
    SHOOT, RESET
  }

  private WPI_VictorSPX m_motorController;
  private PIDController m_pid;
  private Encoder m_encoder;

  int invert = -1;

  public Pusher() {
    m_motorController = new WPI_VictorSPX(PusherConstants.motorId);
    m_motorController.setInverted(true);
    m_pid = new PIDController(PusherConstants.kP, PusherConstants.kI, PusherConstants.kD);
    m_pid.setTolerance(5);
    m_encoder = new Encoder(1, 2);

   // debugging 
   
      SmartDashboard.putNumber(PusherConstants.tableP, PusherConstants.kP);
      SmartDashboard.putNumber(PusherConstants.tableI, PusherConstants.kI);
      SmartDashboard.putNumber(PusherConstants.tableD, PusherConstants.kD);

  }

  public void setMotor(double speed){
    m_motorController.set(ControlMode.PercentOutput, speed);
  };

  public void controllerOff(){
    m_motorController.set(ControlMode.PercentOutput, 0);
  };

  public void controlPower(double power){
      setMotor(power);
  };

  public void controlPID(double setpoint){
    m_motorController.set(MathUtil.clamp(m_pid.calculate(getEncoder(), setpoint), -.5, .5));
  };

  public double getEncoder()
  {
    return m_encoder.getRaw() * invert;
  }

  public void resetEncoder()
  {
    m_encoder.reset();
  }

  public void setPID(){

    double p = SmartDashboard.getNumber(PusherConstants.tableP, PusherConstants.kP);
    double i = SmartDashboard.getNumber(PusherConstants.tableI, PusherConstants.kI);
    double d = SmartDashboard.getNumber(PusherConstants.tableD, PusherConstants.kD);

    m_pid.setPID(p, i, d);
  }

  public boolean isOnSetpoint()
  {
    return m_pid.atSetpoint();
  }

  @Override
  public void periodic(){
     System.out.println("Encoder:" + getEncoder());
     System.out.println("ERROR: " + m_pid.getPositionError());
    setPID();
  }
}