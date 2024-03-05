// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix6.hardware.CANcoder;

//import edu.wpi.first.wpilibj.DigitalInput;
//import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PusherConstants;

public class Pusher extends SubsystemBase {
  private VictorSPX m_motorController;
  private CANcoder m_encoder;
  //private DigitalInput m_beamBreakInput;
  //private DigitalOutput m_beamDigitalOutput;


  public Pusher() {
    m_motorController = new VictorSPX(PusherConstants.motorId);
    m_encoder = new CANcoder(PusherConstants.encoderID);
   // m_beamBreakInput = new DigitalInput(0);
   // m_beamDigitalOutput = new DigitalOutput(4);

    m_motorController.configFactoryDefault();
   // rotations = m_encoder.getPositionSinceBoot();
    // Does this internally set the encoder ticks to 4096?
    //m_motorController.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    //m_motorController.config_kP(0, .3);
    //m_motorController.config_kI(0, 0.);
    //m_motorController.config_kD(0, 0);

    // Will this work as expected to move 100 ticks forward?
    // How can I set a conversion factor to set position in inches?

   // m_beamDigitalOutput.enablePPS(.5);

  }

  public void controllerOn(double speed){
    m_motorController.set(ControlMode.PercentOutput, speed);

  };

  public void controllerOff(){
    m_motorController.set(ControlMode.PercentOutput, 0);

  };

  
  public void setToPosition(double position){
    m_motorController.set(ControlMode.Position, position);
  };

  public CANcoder Encoder(){
    return m_encoder;
  }


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

  // public void moveUntilBreak(double speed)
  // {
  //   if(m_beamBreakInput.get())
  //   {
  //     controllerOff();
  //   }
  //   else
  //   {
  //     controllerOn(speed);
  //   }
  // }

  // public DigitalOutput getBeamOutput()
  // {
  //   return m_beamDigitalOutput;
  // }

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



//   public Command beamBreakOn(double dutyCycle)
//   {
//     return new Command() {
//       @Override
//       public void initialize(){
//         m_beamDigitalOutput.enablePPS(dutyCycle);
//       }
//     };
//   }

//   public Command beamBreakOff()
//   {
//     return new Command(){
//       @Override
//       public void initialize(){
//         m_beamDigitalOutput.disablePWM();
//       }
//   };


// }

//  public SequentialCommandGroup toggleBeamBreak(){
//     return new SequentialCommandGroup(
//       beamBreakOff().withTimeout(.5), beamBreakOn(.5));


//   }


//   @Override 
//   public void periodic(){
    
//     System.out.println(m_beamBreakInput.getChannel());
//     if(!m_beamBreakInput.get())
//     {
//       System.out.println("BEAM BROKEN");
//     }
//     else
//     {
//       System.out.println("not broken");
//     }
//   }
  @Override
  public void periodic(){

    System.out.println(m_encoder.getPosition().getValueAsDouble() * 100);

  }
}