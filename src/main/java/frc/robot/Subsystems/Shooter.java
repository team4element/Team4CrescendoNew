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

  // Power
  public double m_kFAmp = 2.0;
  public double m_kFSpeaker = 25.0;

  public Shooter() {
    Slot0Configs motorConfig = new Slot0Configs();
    
    motorConfig.kS = 0.05; // Add 0.05 V output to overcome static friction
    motorConfig.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    motorConfig.kP = 0.11; // An error of 1 rps results in 0.11 V output
    motorConfig.kI = 0;    // no output for integrated error
    motorConfig.kD = 0;    // no output for error derivative

    // pBinding = new LiveDoubleBinding("Shooter", "P", 0.5, (event) -> {
    //   leftMotorConfig.withKP(event.valueData.value.getDouble());
    // });

    // iBinding = new LiveDoubleBinding("Shooter", "I", 0.0, (event) -> {
    //   leftMotorConfig.withKI(event.valueData.value.getDouble());
    // });

    // dBinding = new LiveDoubleBinding("Shooter", "D", 0.25, (event) -> {
    //   leftMotorConfig.withKD(event.valueData.value.getDouble());
    // });

    // fBinding = new LiveDoubleBinding("Shooter", "F", 0.1, (event) -> {
    //   m_kFSpeaker = event.valueData.value.getDouble();
    // });

    m_request = new VelocityVoltage(0).withSlot(0);

    mLeader.getConfigurator().apply(motorConfig);
    mLeader.setInverted(false);

    m_follower.getConfigurator().apply(motorConfig);

    m_follower.setControl(new Follower(mLeader.getDeviceID(), true));

  }

  @Override
  public void periodic() {
 }

  public void setMotorRPM(double setpoint, boolean PID) {
    if(PID)
    {
       mLeader.setControl(m_request.withVelocity(setpoint).withFeedForward(m_kFAmp));

       System.out.print("Left Error:");
       System.out.print(mLeader.getClosedLoopError());
       System.out.print("| Right Error:");
       System.out.println(m_follower.getClosedLoopError());
    }
    else
    {
      mLeader.set(setpoint);
    }
 }

public void motorsOff()
  {
    mLeader.set(0);
  }
}
