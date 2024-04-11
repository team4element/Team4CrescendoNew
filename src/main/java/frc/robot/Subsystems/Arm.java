// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.VictorSPXConfiguration;
import com.ctre.phoenix.motorcontrol.can.VictorSPXPIDSetConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AngleConstants;
import frc.robot.Constants.ControllerConstants;

/** Add your docs here. */
public class Arm extends SubsystemBase{
    TalonFX m_angleMotor = new TalonFX(AngleConstants.m_angleMotorID);
    VictorSPX m_shootMotor = new VictorSPX(AngleConstants.m_shootMotorID);

    public Arm(){

        VictorSPXPIDSetConfiguration shootConfigs = new VictorSPXPIDSetConfiguration();

        TalonFXConfiguration configs = new TalonFXConfiguration();

        configs.Slot0.kS = 0;  
        configs.Slot0.kV = 0;
        configs.Slot0.kP = 0;
        configs.Slot0.kI = 0;
        configs.Slot0.kD = 0;

        configs.CurrentLimits.SupplyCurrentLimit = AngleConstants.currentLimitAmps;
        configs.CurrentLimits.SupplyCurrentLimitEnable = true;

        m_shootMotor.getPIDConfigs(shootConfigs);

        m_angleMotor.setNeutralMode(NeutralModeValue.Brake);

    }

    public void moveArm(double speed){
       
     if(speed > 0 && m_angleMotor.get() >= AngleConstants.forwardLimit 
       || speed < 0 && m_angleMotor.get() <= AngleConstants.reverseLimit) {

       m_angleMotor.set(0);

     } else{
          m_angleMotor.set(speed);
        }
    
    }

    public void shoot(double speed){
        m_shootMotor.set(ControlMode.Velocity, speed);
    }


    public void take(double speed){
        m_shootMotor.set(ControlMode.Velocity, speed * -1);
    }

    public void motorOff(){
        m_shootMotor.set(ControlMode.Velocity, 0);
    }

    public Command c_manualMovement(){
        return Commands.run(() -> moveArm(ControllerConstants.operatorController.getLeftY()));
    }
}
