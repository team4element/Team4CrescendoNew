// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LiveDoubleBinding;
import frc.robot.Constants.ShooterConstants;
//import edu.wpi.first.math.ArmFeedForward;

public class Shooter extends SubsystemBase {
  private static TalonFX mLeader = new TalonFX(ShooterConstants.m_leftMotorID);
  private static TalonFX m_follower = new TalonFX(ShooterConstants.m_rightMotorID);

  Slot0Configs configsSpeaker = new Slot0Configs();
  LiveDoubleBinding pBinding;
  LiveDoubleBinding iBinding;
  LiveDoubleBinding dBinding;
  LiveDoubleBinding fBinding;

  // Power
  public double m_kFAmp = 2.0;
  public double m_kFSpeaker = 25.0;

  final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

  public Shooter() {
    Slot0Configs leftMotorConfig = new Slot0Configs()
        .withKS(0.0)
        .withKV(0.1)
        .withKP(0.5)
        .withKI(0.0)
        .withKD(0.25);

    pBinding = new LiveDoubleBinding("Shooter", "P", 0.5, (event) -> {
      leftMotorConfig.withKP(event.valueData.value.getDouble());
    });

    iBinding = new LiveDoubleBinding("Shooter", "I", 0.0, (event) -> {
      leftMotorConfig.withKI(event.valueData.value.getDouble());
    });

    dBinding = new LiveDoubleBinding("Shooter", "D", 0.25, (event) -> {
      leftMotorConfig.withKD(event.valueData.value.getDouble());
    });

    fBinding = new LiveDoubleBinding("Shooter", "F", 0.1, (event) -> {
      m_kFSpeaker = event.valueData.value.getDouble();
    });

    mLeader.getConfigurator().apply(leftMotorConfig);
    m_follower.getConfigurator().apply(leftMotorConfig);

    m_follower.setControl(new Follower(mLeader.getDeviceID(), false));
  }

  public void motorsOn(double setpoint) {
    mLeader.setControl(m_request.withVelocity(setpoint).withFeedForward(-m_kFSpeaker));

    System.out.print("Left Error:");
    System.out.print(mLeader.getClosedLoopError());
    System.out.print("| Right Error:");
    System.out.println(m_follower.getClosedLoopError());
  }

  public Command c_runShooter(double setpoint) {
    return startEnd(() -> motorsOn(setpoint), () -> motorsOn(0));
  }
}
