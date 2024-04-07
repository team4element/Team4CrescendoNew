// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

//import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  private static TalonFX m_top = new TalonFX(ShooterConstants.m_topMotorID);
  private static TalonFX m_bottom = new TalonFX(ShooterConstants.m_bottomMotorID);

  final VelocityVoltage m_requestTop;
  final VelocityVoltage m_requestBot;

  public Shooter() {
    // SmartDashboard.putNumber(ShooterConstants.tableTopRPM, ShooterConstants.rpmTopHigh);
    // SmartDashboard.putNumber(ShooterConstants.tableBotRPM, ShooterConstants.rpmBotHigh);

    TalonFXConfiguration config = new TalonFXConfiguration();
    
    config.Slot0.kS = 0.05; // Add 0.05 V output to overcome static friction
    config.Slot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    config.Slot0.kP = 0.15; // An error of 1 rps results in 0.11 V output
    config.Slot0.kI = 0;    // no output for integrated error
    config.Slot0.kD = 0;    // no output for error derivative

    config.CurrentLimits.SupplyCurrentLimit = ShooterConstants.currentLimitAmps;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    m_requestTop = new VelocityVoltage(0).withSlot(0);
    m_requestBot = new VelocityVoltage(0).withSlot(0);

    m_top.getConfigurator().apply(config);
    m_bottom.getConfigurator().apply(config);
    
    m_top.setInverted(true);
    m_bottom.setInverted(true);
  }

  //debugging 

  // public void updateHighRpmFromTable()
  // {
  //   ShooterConstants.rpmTopHigh = SmartDashboard.getNumber(ShooterConstants.tableTopRPM, 0);
  //   ShooterConstants.rpmBotHigh = SmartDashboard.getNumber(ShooterConstants.tableBotRPM, 0);
  // }

  //  public void updateLowRpmFromTable()
  // {
  //   ShooterConstants.rpmTop = SmartDashboard.getNumber(ShooterConstants.tableTopRPM, 0);
  //   ShooterConstants.rpmBotHigh = SmartDashboard.getNumber(ShooterConstants.tableBotRPM, 0);
  // }

  @Override
  public void periodic() {
   // printEncoderError();
   // updateHighRpmFromTable();
  }

  /**
   * Sets the motor to a desired speed
   * @param setpoint The desired speed in rpm
   */
  public void setMotorRPM(double setpointTop, double setpointBot) {
      m_top.setControl(m_requestTop.withVelocity(setpointTop).withFeedForward(.5));
      m_bottom.setControl(m_requestBot.withVelocity(setpointBot).withFeedForward(.5));
 }

 //debugging 

//  public void printEncoderError()
//  {
//  //   System.out.print("Top Error:");
//    // System.out.print(m_top.getClosedLoopError());

//    // System.out.print("Bot Error:");
//    // System.out.print(m_bottom.getClosedLoopError());
//  }

public void motorsOff()
  {
    m_top.set(0);
    m_bottom.set(0);
  }
}
