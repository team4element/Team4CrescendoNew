// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LiveDoubleBinding;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  private static TalonFX mLeader = new TalonFX(ShooterConstants.m_leftMotorID);
  private static TalonFX m_follower = new TalonFX(ShooterConstants.m_rightMotorID);

  final VelocityVoltage m_request;

  Slot0Configs configsSpeaker = new Slot0Configs();
  LiveDoubleBinding pBinding;
  LiveDoubleBinding iBinding;
  LiveDoubleBinding dBinding;
  LiveDoubleBinding fBinding;

  public Shooter() {
    Slot0Configs motorConfig = new Slot0Configs();
    
    motorConfig.kS = 0.05; // Add 0.05 V output to overcome static friction
    motorConfig.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    motorConfig.kP = 0.15; // An error of 1 rps results in 0.11 V output
    motorConfig.kI = 0;    // no output for integrated error
    motorConfig.kD = 0;    // no output for error derivative

    m_request = new VelocityVoltage(0).withSlot(0);

    mLeader.getConfigurator().apply(motorConfig);
    mLeader.setInverted(false);

    m_follower.getConfigurator().apply(motorConfig);

    m_follower.setControl(new Follower(mLeader.getDeviceID(), true));

  }

  @Override
  public void periodic() {
    // printEncoderError();
  }

  /**
   * Sets the motor to a desired speed
   * @param setpoint The desired speed in rpm
   */
  public void setMotorRPM(double setpoint) {
       mLeader.setControl(m_request.withVelocity(setpoint).withFeedForward(.5));
 }

 /**
  * Prints the velocity error with the motors
  */
 public void printEncoderError()
 {
    System.out.print("Error:");
    System.out.print(mLeader.getClosedLoopError());
 }


 /**
  * Turns off the motors
  */
public void motorsOff()
  {
    mLeader.set(0);
  }
}
