// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;


import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ControllerConstants;

/** Add your docs here. */
public class Arm extends SubsystemBase{
    TalonFX m_angleMotor; 
    TalonFX m_shootMotor; 
    int m_invert;

    double m_angle;

    TalonFXConfiguration angleConfigs;

    PositionVoltage m_request;

    public Arm(){

        m_angleMotor = new TalonFX(ArmConstants.m_angleMotorID);
        m_shootMotor = new TalonFX(ArmConstants.m_shootMotorID);
            m_invert = -1;

        angleConfigs = new TalonFXConfiguration();
        
        // SmartDashboard.putNumber(ArmConstants.tableP, ArmConstants.kP);
        // SmartDashboard.putNumber(ArmConstants.tableI, ArmConstants.kI);
        // SmartDashboard.putNumber(ArmConstants.tableD, ArmConstants.kD);

        TalonFXConfiguration shootConfigs = new TalonFXConfiguration();

        shootConfigs.Slot0.kP = 0;
        shootConfigs.Slot0.kI = 0;
        shootConfigs.Slot0.kD = 0;

        angleConfigs.CurrentLimits.SupplyCurrentLimit = ArmConstants.angleCurrentLimitAmps;
        angleConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

        shootConfigs.CurrentLimits.SupplyCurrentLimit = ArmConstants.shootCurrentLimitAmps;
        shootConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

        m_angleMotor.getConfigurator().apply(angleConfigs);
        m_shootMotor.getConfigurator().apply(shootConfigs);

        m_angleMotor.setNeutralMode(NeutralModeValue.Brake);

        m_angle = 0;
    }

    public void moveArm(double speed){

     if(speed + m_invert > 0 && m_angle >= ArmConstants.forwardLimit
       || speed + m_invert < 0 && m_angle <= ArmConstants.reverseLimit) {

        m_angleMotor.set(0);

     } else{
        m_angleMotor.set(speed * m_invert);
        }
    }

    public void setSpeed(double speed){
        m_shootMotor.set(speed);
    }

    public void amp(double setpoint) {
        m_angleMotor.setControl(m_request.withPosition(setpoint));
    }

    public void motorOff(){
        m_shootMotor.set(0);
    }

    public void angleOff(){
        m_angleMotor.set(0);
    }

    public void zeroEncoder(){
        m_angleMotor.setPosition(0, 1);
    }

    // public void setPID()
//    {
//      double p = SmartDashboard.getNumber(ArmConstants.tableP, ArmConstants.kP);
//      double i = SmartDashboard.getNumber(ArmConstants.tableI, ArmConstants.kI);
//      double d = SmartDashboard.getNumber(ArmConstants.tableD, ArmConstants.kD);

//      angleConfigs.Slot0.kP = p;
//      angleConfigs.Slot0.kI = i;
//      angleConfigs.Slot0.kD = d;

//     m_angleMotor.getConfigurator().apply(angleConfigs);
//    }

    public double getCurrentPosition() {
        return m_angleMotor.getPosition().getValueAsDouble();
      }

    public Command c_manualMovement(){
        return Commands.run(() -> moveArm(ControllerConstants.operatorController.getLeftY() * .3), this);
    }

    //     public Command c_manualMovement2(){
    //     return Commands.run(() -> moveArm(ControllerConstants.driveController.getLeftTriggerAxis() * .3), this);
    // }
    

     @Override
     public void periodic(){
        m_angle = m_angleMotor.getPosition().getValue();
    //     System.out.println("ARM ANGLE: " + m_angle);
     //setPID();
    // System.out.println("angle motor: " + m_angleMotor.getPosition());
    }
}
