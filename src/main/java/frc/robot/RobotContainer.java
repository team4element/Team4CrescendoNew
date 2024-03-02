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
import frc.robot.Commands.ExtendClimb;
import frc.robot.Commands.PullUp;
import frc.robot.Commands.Push;
import frc.robot.Commands.Shoot;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.ConveyorConstants;
import frc.robot.Constants.PusherConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Conveyor;
import frc.robot.Subsystems.Pusher;
import frc.robot.Subsystems.Shooter;

public class RobotContainer {
  SendableChooser<Command> autoChooser;

  // final ColorMatch m_colorMatcher = new ColorMatch();

  // Subsystems
  public static final CommandSwerveDrivetrain m_driveTrain = new CommandSwerveDrivetrain();
  public static final Conveyor m_conveyor = new Conveyor();
  public static final Shooter m_shooter = new Shooter();
  public static final Pusher m_pusher = new Pusher();
  public static final Climber m_climber = new Climber();

  private final Telemetry logger;

  public RobotContainer() {

    // NamedCommands.registerCommand("shoot", m_shooter.setMotorRPM(0, false));
    NamedCommands.registerCommand("Push And Shoot High",
        pushAndShoot(
        ShooterConstants.rmpHigh, ShooterConstants.timeoutHigh, PusherConstants.highSpeed));
    NamedCommands.registerCommand("Push And Shoot Low",
        pushAndShoot(
        ShooterConstants.rmpLow, ShooterConstants.timeoutLow, PusherConstants.lowSpeed));
    NamedCommands.registerCommand("Intake", 
       m_conveyor.c_runBoth(
      Conveyor.Direction.INTAKE, 0.8));
    
    //.withTimeout(.5));

    autoChooser = AutoBuilder.buildAutoChooser(); // Defaults to an empty command.

    logger = new Telemetry(m_driveTrain.maxSpeedSupplier.get());

    SmartDashboard.putData("Auto Chooser", autoChooser);
    m_driveTrain.registerTelemetry(logger::telemeterize);
    configureBindings();
    m_driveTrain.setDefaultCommand(m_driveTrain.c_OpenLoopDrive());

  }

  public void onAutonInit() {
    m_driveTrain.seedFieldRelative();
    m_pusher.zeroEncoder();
  }

  public void onTeleopInit() {
    // m_driveTrain.seedFieldRelative();
  }

  private void configureBindings() {
    ControllerConstants.driveController.y().whileTrue(m_driveTrain.c_cardinalLock(0));
    ControllerConstants.driveController.x().whileTrue(m_driveTrain.c_cardinalLock(90));
    ControllerConstants.driveController.a().whileTrue(m_driveTrain.c_cardinalLock(180));
    ControllerConstants.driveController.b().whileTrue(m_driveTrain.c_cardinalLock(270));
    ControllerConstants.driveController.leftBumper().onTrue(m_driveTrain.c_seedFieldRelative());
    
    ControllerConstants.driveController.povDown().whileTrue(new PullUp(m_climber));
    ControllerConstants.driveController.povUp().whileTrue(new ExtendClimb(m_climber));

    ControllerConstants.operatorController.leftBumper()
        .whileTrue(m_conveyor.c_runBoth(Conveyor.Direction.OUTTAKE, ConveyorConstants.conveyorSpeed));
    ControllerConstants.operatorController.rightBumper()
        .whileTrue(m_conveyor.c_runBoth(Conveyor.Direction.INTAKE, ConveyorConstants.conveyorSpeed));
    ControllerConstants.operatorController.y()
        .toggleOnTrue(pushAndShoot(ShooterConstants.rmpHigh, ShooterConstants.timeoutHigh, PusherConstants.highSpeed));
    ControllerConstants.operatorController.b().toggleOnTrue(
        pushAndShoot(ShooterConstants.rmpMedium, ShooterConstants.timeoutMedium, PusherConstants.medSpeed));
    ControllerConstants.operatorController.a()
        .toggleOnTrue(pushAndShoot(ShooterConstants.rmpLow, ShooterConstants.timeoutLow, PusherConstants.lowSpeed));
    ControllerConstants.operatorController.x().whileTrue(new Shoot(m_shooter, ShooterConstants.rmpReverse));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  private SequentialCommandGroup pushAndShoot(double rpm, double timeout, double pusherSpeed) {
    return new SequentialCommandGroup(new Shoot(m_shooter, rpm).withTimeout(ShooterConstants.rampUpTime),
        new ParallelCommandGroup(
            new Shoot(m_shooter, rpm),
            new Push(m_pusher, pusherSpeed)).withTimeout(timeout));
  }

  //private SequentialCommandGroup climb() {
 //   return new SequentialCommandGroup( new ExtendClimb(m_climber).withTimeout(5), new PullUp(m_climber));
 // }

}
