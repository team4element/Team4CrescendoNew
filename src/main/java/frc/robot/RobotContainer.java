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
import frc.robot.Commands.Climb;
import frc.robot.Commands.Push;
import frc.robot.Commands.Shoot;
import frc.robot.Commands.climbToSetpoint;
import frc.robot.Commands.getPusherToSetpoint;
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

  // private final Telemetry logger;

  public RobotContainer() {

    // NamedCommands.registerCommand("shoot", m_shooter.setMotorRPM(0, false));
    NamedCommands.registerCommand("Push And Shoot High",
        pushAndShoot(
        ShooterConstants.rpmTopHigh, ShooterConstants.rpmBotHigh, ShooterConstants.timeoutHigh));
    NamedCommands.registerCommand("Push And Shoot Low",
        pushAndShoot(
        ShooterConstants.rpmTopLow, ShooterConstants.rpmBotLow, ShooterConstants.timeoutLow));
    NamedCommands.registerCommand("Intake",
       m_conveyor.c_runBoth(
      Conveyor.Direction.INTAKE, 0.8).withTimeout(2.5));

    //.withTimeout(.5));
    autoChooser = AutoBuilder.buildAutoChooser(); // Defaults to an empty command.

    // logger = new Telemetry(m_driveTrain.maxSpeedSupplier.get());

    SmartDashboard.putData("Auto Chooser", autoChooser);
    // m_driveTrain.registerTelemetry(logger::telemeterize);
    configureBindings();
    m_driveTrain.setDefaultCommand(m_driveTrain.c_OpenLoopDrive());

  }

  public void onAutonInit() {
    m_driveTrain.seedFieldRelative();
    m_pusher.resetEncoder();
  }

  public void onTeleopInit() {
    m_driveTrain.seedFieldRelative();
    m_pusher.resetEncoder();
    m_climber.resetMotor();
  }

  private void configureBindings() {
    // ControllerConstants.driveController.y().whileTrue(m_driveTrain.c_cardinalLock(0));
    // ControllerConstants.driveController.x().whileTrue(m_driveTrain.c_cardinalLock(90));
    // ControllerConstants.driveController.a().whileTrue(m_driveTrain.c_cardinalLock(180));
    // ControllerConstants.driveController.b().whileTrue(m_driveTrain.c_cardinalLock(270));
    ControllerConstants.driveController.leftBumper().onTrue(m_driveTrain.c_seedFieldRelative());
    ControllerConstants.driveController.rightBumper().onTrue(m_driveTrain.c_invertControls());

    ControllerConstants.driveController.povUp().whileTrue(new climbToSetpoint(m_climber, 10));
    ControllerConstants.driveController.povDown().whileTrue(new climbToSetpoint(m_climber, 0));

    ControllerConstants.operatorController.leftBumper()
        .whileTrue(m_conveyor.c_runBoth(Conveyor.Direction.OUTTAKE, ConveyorConstants.conveyorSpeed));
    ControllerConstants.operatorController.rightBumper()
        .whileTrue(m_conveyor.c_runBoth(Conveyor.Direction.INTAKE, ConveyorConstants.conveyorSpeed));
    ControllerConstants.operatorController.y()
        .toggleOnTrue( pushAndShoot(ShooterConstants.rpmTopHigh, ShooterConstants.rpmBotHigh, ShooterConstants.timeoutHigh));
    // ControllerConstants.operatorController.b()
    //     .toggleOnTrue(pushAndShoot(ShooterConstants.rmpMedium, 0, ShooterConstants.timeoutMedium));
    ControllerConstants.operatorController.a()
        .toggleOnTrue(pushAndShoot(ShooterConstants.rpmTopLow, ShooterConstants.rpmBotLow, ShooterConstants.timeoutLow));
    ControllerConstants.operatorController.x().onTrue(new getPusherToSetpoint(m_pusher, PusherConstants.encoderPosition).withTimeout(1.5));
    ControllerConstants.operatorController.povUp().whileTrue(new Push(m_pusher, PusherConstants.lowSpeed));
    ControllerConstants.operatorController.povDown().whileTrue(new Push(m_pusher,-PusherConstants.lowSpeed));
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
}
