// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PusherContants;

public class Pusher extends SubsystemBase {
    private VictorSPX m_motorController;
    private AnalogPotentiometer pot;

  public Pusher() {
    m_motorController = new VictorSPX(PusherContants.motorId);
    pot = new AnalogPotentiometer(PusherContants.potId);

    m_motorController.config_kP(0, .3);
    m_motorController.config_kI(0, 0);
    m_motorController.config_kD(0, 0);

  }

  public void controllerOn(double speed){
    m_motorController.set(ControlMode.PercentOutput, speed);
  };

  public void controllerOff(){
    m_motorController.set(ControlMode.PercentOutput, 0);
  };

  public double getPot()
  {
     return pot.get();
  }

  public void movePusherToAngle(int angle)
  {

    double maxGravityFF = 0.07;
    int kMeasuredPosHorizontal = 840; //Position measured when arm is horizontal
    double kTicksPerDegree = 4096 / 360; //Sensor is 1:1 with arm rotation
    double currentPos = getPot();
    double degrees = (currentPos - kMeasuredPosHorizontal) / kTicksPerDegree;
    double radians = java.lang.Math.toRadians(degrees);
    double cosineScalar = java.lang.Math.cos(radians);

    m_motorController.set(VictorSPXControlMode.MotionMagic, angle, DemandType.ArbitraryFeedForward, maxGravityFF * cosineScalar);


  }
}
