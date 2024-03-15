// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PusherConstants;
//import static org.junit.jupiter.api.Assertions.assertEquals;

public class Pusher extends SubsystemBase {

  public enum Direction {
    SHOOT, RESET
  }

  private VictorSPX m_motorController;
  private CANcoder m_encoder;
  private int m_resets;


  public Pusher() {
    m_motorController = new VictorSPX(PusherConstants.motorId);
    m_encoder = new CANcoder(PusherConstants.encoderID);

    m_motorController.configFactoryDefault();
    m_motorController.config_kP(0, .3);
    m_motorController.config_kI(0, 0);
    m_motorController.config_kD(0, 0);

    CANcoderConfiguration configs = new CANcoderConfiguration();

    configs.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    configs.MagnetSensor.MagnetOffset = 0.402099609375;
    configs.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;

    m_encoder.getConfigurator().apply(configs);

    m_resets = 0;
  }

  public void setMotor(double speed){
    m_motorController.set(ControlMode.PercentOutput, speed);

  };
  

  public void controllerOff(){
    m_motorController.set(ControlMode.PercentOutput, 0);

  };

  public void setToPosition(double position){

    m_motorController.set(ControlMode.Position, position);

  };

  public void zeroEncoder(){
    m_encoder.setPosition(0);
  }

  public double getEncoderPosition(){
    return ((m_encoder.getAbsolutePosition().getValueAsDouble()
    -PusherConstants.encoderOffset)*100);
  }

  public Command c_pushContinuously(double speed){
    return new Command() {
      @Override
      public void execute() {
        setMotor(speed);
      }
      @Override
      public void end(boolean interrupted) {
        controllerOff();
      }
    };
  }

  @Override
  public void periodic(){
<<<<<<< Updated upstream
    System.out.println("Original encoder value: " + (m_encoder.getAbsolutePosition().getValueAsDouble() -PusherConstants.encoderOffset)* 100);
=======
    if(m_encoder.getPosition().getValueAsDouble() == 0)
      ++m_resets;
    SmartDashboard.putNumber("Encoder Position", m_encoder.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Encoder Absolute Position", m_encoder.getAbsolutePosition().getValueAsDouble());
    SmartDashboard.putNumber("resets", m_resets);

>>>>>>> Stashed changes
  }
}
