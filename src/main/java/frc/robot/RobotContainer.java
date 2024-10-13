// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.BlinkLeds;
import frc.robot.Commands.Climb;
import frc.robot.Commands.Push;
import frc.robot.Commands.Shoot;
import frc.robot.Commands.climbToSetpoint;
import frc.robot.Commands.ShootWithArm;
import frc.robot.Commands.armToAmp;
import frc.robot.Commands.getPusherToSetpoint;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.ConveyorConstants;
import frc.robot.Constants.PusherConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Conveyor;
import frc.robot.Subsystems.Pusher;
import frc.robot.Subsystems.Shooter;

public class RobotContainer {
  SendableChooser<Command> autoChooser;

  // Subsystems
  public static final CommandSwerveDrivetrain m_driveTrain = new CommandSwerveDrivetrain();
  public static final Conveyor m_conveyor = new Conveyor();
  public static final Shooter m_shooter = new Shooter();
  public static final Pusher m_pusher = new Pusher();
  public static final Climber m_climber = new Climber();
  public static final Arm m_arm = new Arm();
  public static final VisionTracking = new VisionTracking();

  public RobotContainer() {

    NamedCommands.registerCommand("Push And Shoot High",
        pushAndShoot(
        ShooterConstants.rpmTopHigh, ShooterConstants.rpmBotHigh, ShooterConstants.timeoutHigh));
    NamedCommands.registerCommand("Push And Shoot Low",
        pushAndShoot(
        ShooterConstants.rpmTopLow, ShooterConstants.rpmBotLow, ShooterConstants.timeoutLow));
    NamedCommands.registerCommand("Intake",
       m_conveyor.c_runBoth(
      Conveyor.Direction.INTAKE, 0.8).withTimeout(2.5));

    autoChooser = AutoBuilder.buildAutoChooser(); // Defaults to an empty command.

    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureBindings();
    m_driveTrain.setDefaultCommand(m_driveTrain.c_OpenLoopDrive());
    m_arm.setDefaultCommand(m_arm.c_manualMovement());
    //m_arm.setDefaultCommand(m_arm.c_manualMovement2());
  }

  public void onAutonInit() {
    m_driveTrain.seedFieldRelative();
    m_pusher.resetEncoder();
  }

  public boolean getLimitSwitchFront(){
     return m_conveyor.getLimitSwitchFront();
  }

  public boolean getLimitSwitchBack(){
    return m_conveyor.getLimitSwitchBack();
  }

  public void onTeleopInit() {
    m_driveTrain.seedFieldRelative();
    m_pusher.resetEncoder();
    m_climber.resetMotor();
    switchLedColor();
  }

  public void switchLedColor(){
    new BlinkLeds(m_conveyor, m_conveyor.lastColor()).schedule();
  }

  private void configureBindings() {
     ControllerConstants.driveController.y().whileTrue(new Climb(m_climber, -.5));
     //for demo only 

     ControllerConstants.driveController.a().whileTrue(m_driveTrain.c_cardinalLock(60));
    ControllerConstants.driveController.leftBumper().onTrue(m_driveTrain.c_seedFieldRelative());
   // ControllerConstants.driveController.rightBumper().onTrue(m_driveTrain.c_invertControls());

     //ControllerConstants.driveController.y()
        //.toggleOnTrue( pushAndShoot(ShooterConstants.rpmTopHigh, ShooterConstants.rpmBotHigh, ShooterConstants.timeoutHigh));
    //  ControllerConstants.driveController.a()
        // .toggleOnTrue(pushAndShoot(ShooterConstants.rpmTopLow, ShooterConstants.rpmBotLow, ShooterConstants.timeoutLow));
    //  ControllerConstants.driveController.b().whileTrue(new ShootWithArm(m_arm, -.6));
     // ControllerConstants.driveController.x().whileTrue(new ShootWithArm(m_arm, .2));

     ControllerConstants.driveController.povUp().whileTrue(new climbToSetpoint(
       m_climber, ClimberConstants.setpointUp));
     ControllerConstants.driveController.povDown().whileTrue(new Climb(m_climber, .5));
    // ControllerConstants.driveController.povUp()
    // .whileTrue(new Push(m_pusher, PusherConstants.lowSpeed));
    // ControllerConstants.driveController.povDown()
    // .whileTrue(new Push(m_pusher,-PusherConstants.lowSpeed));
    // ControllerConstants.driveController.povLeft()
    // .whileTrue(m_conveyor.c_runBoth(Conveyor.Direction.OUTTAKE, ConveyorConstants.conveyorSpeed));
    // ControllerConstants.driveController.povRight()
    // .whileTrue(m_conveyor.c_runBoth(Conveyor.Direction.INTAKE, ConveyorConstants.conveyorSpeed));


    ControllerConstants.operatorController.leftBumper()
         .whileTrue(m_conveyor.c_runBoth(Conveyor.Direction.OUTTAKE, ConveyorConstants.conveyorSpeed));
    ControllerConstants.operatorController.rightBumper()
         .whileTrue(m_conveyor.c_runBoth(Conveyor.Direction.INTAKE, ConveyorConstants.conveyorSpeed));
     ControllerConstants.operatorController.y()
         .toggleOnTrue( pushAndShoot(ShooterConstants.rpmTopHigh, ShooterConstants.rpmBotHigh, ShooterConstants.timeoutHigh));
      //  ControllerConstants.operatorController.b()
      //     .toggleOnTrue(pushAndShoot(ShooterConstants.rpmTopTrapv4, ShooterConstants.rpmBotTrapv4, ShooterConstants.timeoutMedium));
              ControllerConstants.operatorController.a()
          .toggleOnTrue(pushAndShoot(ShooterConstants.rpmTopLow, ShooterConstants.rpmBotLow, ShooterConstants.timeoutLow));
      // ControllerConstants.operatorController.x()
      // .toggleOnTrue( pushAndShoot(ShooterConstants.rpmTopTrapv2, ShooterConstants.rpmBotTrapv2, ShooterConstants.timeoutHigh));
  //  .onTrue(new getPusherToSetpoint(m_pusher, PusherConstants.encoderPosition).withTimeout(1.5));
     ControllerConstants.operatorController.povUp().whileTrue(new Push(m_pusher, PusherConstants.lowSpeed));
     ControllerConstants.operatorController.povDown().whileTrue(new Push(m_pusher,-PusherConstants.lowSpeed));
     ControllerConstants.operatorController.povLeft().toggleOnTrue(pushAndShoot(ShooterConstants.rpmTopTrapv2, ShooterConstants.rpmTopTrapv2, ShooterConstants.timeoutMedium));
     ControllerConstants.operatorController.povRight().toggleOnTrue(pushAndShoot(ShooterConstants.rpmTopTrapv3, ShooterConstants.rpmTopTrapv3, ShooterConstants.timeoutMedium));

     //ControllerConstants.operatorController.b().whileTrue(new ShootWithArm(m_arm, -.6));
     ControllerConstants.operatorController.x().whileTrue(new ShootWithArm(m_arm, .2));

      ControllerConstants.operatorController.b()
          .toggleOnTrue (ampShot(ShooterConstants.rpmTopLow, ShooterConstants.rpmBotLow, ShooterConstants.timeoutLow, -.9, ArmConstants.timeout));
         // .toggleOnTrue (ampShoot(ShooterConstants.rpmTopLow, ShooterConstants.rpmBotLow, 
         // ShooterConstants.timeoutLow, 1, -.6));

  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  private SequentialCommandGroup pushAndShoot(double rpmTop, double rpmBot, double timeout) {
    return new SequentialCommandGroup(new Shoot(m_shooter, rpmTop, rpmBot).withTimeout(ShooterConstants.rampUpTime),
        new ParallelCommandGroup(
            new Shoot(m_shooter, rpmTop, rpmBot),
            new getPusherToSetpoint(m_pusher, PusherConstants.encoderPosition))
            .withTimeout(timeout));
  }



private SequentialCommandGroup ampShot(double rpmTop, double rpmBot, double timeout, double armSpeed, double timeout2) {
    return new SequentialCommandGroup(new ShootWithArm(m_arm, -.6).withTimeout(ArmConstants.ramp),
        new ParallelCommandGroup(
            new ShootWithArm(m_arm, -.9),
            new Shoot(m_shooter, rpmTop, rpmBot),
            new getPusherToSetpoint(m_pusher, PusherConstants.encoderPosition))
            .withTimeout(1.5));
}
  //  private SequentialCommandGroup ampShoot(int rpmTop, int rpmBot, double timeout, double setpoint, double armSpeed) {
  //    return new SequentialCommandGroup(new armToAmp(m_arm, setpoint),
  //      new ParallelCommandGroup(
  //       pushAndShoot(rpmTop, rpmBot, timeout),
  //       new ShootWithArm(m_arm, -.6))
  //       .withTimeout(3));
  //  }
}
