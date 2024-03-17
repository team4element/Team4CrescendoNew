// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.PusherConstants;

public class Pusher extends SubsystemBase {

  public enum Direction {
    SHOOT, RESET
  }

  private VictorSPX m_motorController;
  private AnalogPotentiometer m_pot;


  public Pusher() {
    m_motorController = new VictorSPX(PusherConstants.motorId);
    m_pot = new AnalogPotentiometer(PusherConstants.potID, 360, 0);

    m_motorController.configFactoryDefault();
    m_motorController.config_kP(0, .3);
    m_motorController.config_kI(0, 0.);
    m_motorController.config_kD(0, 0);

  }

  public void setMotor(double speed){
    m_motorController.set(ControlMode.PercentOutput, speed);
  };

  public void controllerOff(){
    m_motorController.set(ControlMode.PercentOutput, 0);
  };

  public void setToPosition(double position, double tolerance){
    double error = position - m_pot.get();
    while (Math.abs(error) > tolerance) {
      double power = PusherConstants.kP * error;
      m_motorController.set(ControlMode.PercentOutput, power);
    }
  
  };

  public double getDegree()
  {
    return m_pot.get();
  }

  @Override
  public void periodic(){
    System.out.println("Potentiometer:" + getDegree());
    System.out.println("Victor: " + m_motorController.getSelectedSensorPosition());
  }
}