// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LiveDoubleBinding;
import frc.robot.Constants.ShooterConstants;
//import edu.wpi.first.math.ArmFeedForward;

public class Shooter extends SubsystemBase {


  private static TalonFX leftMotor = new TalonFX(ShooterConstants.m_leftMotorID);
  private static TalonFX rightMotor = new TalonFX(ShooterConstants.m_rightMotorID);

  Slot0Configs configsAmp = new Slot0Configs();
  Slot0Configs configsSpeaker = new Slot0Configs();

  //Power
  public static final double m_kFAmp = 2.0;
  public static final double m_kFSpeaker = 25.0;
  
  LiveDoubleBinding pBinding = new LiveDoubleBinding("Shooter", "P", 0.5, (event) -> {
    configsAmp.withKP(event.valueData.value.getDouble());
  });

  LiveDoubleBinding iBinding = new LiveDoubleBinding("Shooter", "I", 0.0, (event) -> {
    configsAmp.withKI(event.valueData.value.getDouble());
  });

  LiveDoubleBinding dBinding = new LiveDoubleBinding("Shooter", "D", 0.25, (event) -> {
    configsAmp.withKD(event.valueData.value.getDouble());
  });

  final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

  public Shooter() {
    configsAmp.kP = 0.5;
    configsAmp.kI = 0.0;
    configsAmp.kD = 0.25;
    configsAmp.kV = 0.1;
    configsAmp.kS = 0.0;
    configsAmp.kA = 0.0;
    configsAmp.kG = 0.0;
    //10 percent

    configsSpeaker.kP = 3;
    configsSpeaker.kI = 0.0;
    configsSpeaker.kD = 0.5;
    configsSpeaker.kV = 0.1;
    configsSpeaker.kS = 0.0;
    configsSpeaker.kA = 0.0;
    configsSpeaker.kG = 0.0;

    motorsOff();

  }

  @Override
  public void periodic() {
 }

 public void motorsOn(double setpoint) {
    leftMotor.setControl(m_request.withVelocity(setpoint).withFeedForward(-m_kFSpeaker));
    rightMotor.setControl(m_request.withVelocity(setpoint).withFeedForward(-m_kFSpeaker));

    System.out.print("Left Error:");
    System.out.print(leftMotor.getClosedLoopError());
    System.out.print("| Right Error:");
    System.out.println(rightMotor.getClosedLoopError());

   // leftMotor.setVoltage(pid.calculate(encoder.getDistance(), setpoint) + feedforward);

 }


  public void motorsOff() {
    leftMotor.set(0);
   rightMotor.set(0);

  }
}
