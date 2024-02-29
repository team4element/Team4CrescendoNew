// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ElementUnits;
import frc.robot.Constants.PusherConstants;

public class Pusher extends SubsystemBase {
    private VictorSPX m_motorController;
   // private DutyCycleEncoder m_encoder;

  public Pusher() {
    m_motorController = new VictorSPX(PusherConstants.motorId);
   // m_encoder = new DutyCycleEncoder(PusherConstants.encoderID);

    m_motorController.config_kP(0, .3);
    m_motorController.config_kI(0, 0);
    m_motorController.config_kD(0, 0);

  }

  public void controllerOn(double speed){
    m_motorController.set(ControlMode.PercentOutput, speed);
   // m_encoder.setDistancePerRotation(20);
  };

  public void controllerOff(){
    m_motorController.set(ControlMode.PercentOutput, 0);
  };

  //public double setAngleToTicks(double angle){
   // return ElementUnits.rotationsToTicks((angle/360) / PusherConstants.kGearRatio, PusherConstants.kPulsePerRev);

 // };

 // public double getEncoderDistance(){
   // return m_encoder.getDistance();

  //};
}
