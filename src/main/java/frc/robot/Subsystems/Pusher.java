// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.VictorSPXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PusherConstants;

public class Pusher extends SubsystemBase {
  private VictorSPX m_motorController;
  //private CANcoder m_encoder;
  private double rotations;
  
  public Pusher() {
    m_motorController = new VictorSPX(PusherConstants.motorId);
   // m_encoder = new CANcoder(PusherConstants.encoderID);
    m_motorController.configFactoryDefault();
   // rotations = m_encoder.getPositionSinceBoot();
    // Does this internally set the encoder ticks to 4096?
    // m_motorController.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
   // m_motorController.config_kP(0, .3);
   // m_motorController.config_kI(0, 0.);
   // m_motorController.config_kD(0, 0);

    // Will this work as expected to move 100 ticks forward?
    // How can I set a conversion factor to set position in inches?

  }

  public void controllerOn(double speed){
    m_motorController.set(ControlMode.PercentOutput, speed);
    System.out.println(rotations);
    //m_encoder.setPosition(1);
    //m_encoder.setDistancePerRotation(20);
  };

  public void controllerOff(){
    m_motorController.set(ControlMode.PercentOutput, 0);

  };

  public void setToPosition(double position){
    m_motorController.set(ControlMode.Position, position);
  };

  public void zeroEncoder(){
    m_motorController.setSelectedSensorPosition(0);
  };

  public void Encoder(){

   if (rotations == 1)
   {
      m_motorController.setSelectedSensorPosition(0);
   };
    
    
    //the value is in rotations

  };

  public Command c_pushContinuously(double speed){
    return new Command() {
      @Override
      public void execute() {
        controllerOn(speed);
      }
      @Override
      public void end(boolean interrupted) {
        controllerOff();
      }
    };
  }

  public enum Direction {
    SHOOT, RESET
  }

  // public Command c_pushToPosition(double position){
  //   // m_motorController.();
  //   return new Command() {
  //     @Override
  //     public void execute() {
  //       setToPosition(position);
  //     }
  //     @Override
  //     public void end(boolean interrupted) {
  //       controllerOff();
  //     }
  //   };
  // }

}
