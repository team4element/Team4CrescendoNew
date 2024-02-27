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
import frc.robot.Commands.Push;
import frc.robot.Commands.Shoot;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.ConveyorConstants;
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

  // TODO: Should this go inside drivetrain class or should it be abstracted to
  // RobotState class?
  private final Telemetry logger = new Telemetry(m_driveTrain.maxSpeedSupplier.get());

  public RobotContainer() {
    autoChooser = AutoBuilder.buildAutoChooser(); // Defaults to an empty command.
    
    // NamedCommands.registerCommand("shoot", m_shooter.setMotorRPM(0, false););
    NamedCommands.registerCommand("intake", m_conveyor.c_runBoth(Conveyor.Direction.INTAKE, 0.8));

    SmartDashboard.putData("Auto Chooser", autoChooser);
    m_driveTrain.registerTelemetry(logger::telemeterize);
    configureBindings();
    m_driveTrain.setDefaultCommand(m_driveTrain.c_OpenLoopDrive());
  }

  public void onInit() {
    m_driveTrain.seedFieldRelative();
  }

  private void configureBindings() {
    ControllerConstants.driveController.y().whileTrue(m_driveTrain.c_cardinalLock(0));
    ControllerConstants.driveController.x().whileTrue(m_driveTrain.c_cardinalLock(90));
    ControllerConstants.driveController.a().whileTrue(m_driveTrain.c_cardinalLock(180));
    ControllerConstants.driveController.b().whileTrue(m_driveTrain.c_cardinalLock(270));
    ControllerConstants.driveController.leftBumper().onTrue(m_driveTrain.c_seedFieldRelative());

    // ControllerConstants.operatorController.y().whileTrue(new Bottom(m_conveyor));
    // ControllerConstants.operatorController.x().whileTrue(new Top(m_conveyor));
    ControllerConstants.operatorController.leftBumper()
      .whileTrue(m_conveyor.c_runBoth(Conveyor.Direction.OUTTAKE, ConveyorConstants.conveyorSpeed));
    ControllerConstants.operatorController.rightBumper()
        .whileTrue(m_conveyor.c_runBoth(Conveyor.Direction.INTAKE, ConveyorConstants.conveyorSpeed));
    ControllerConstants.operatorController.b().whileTrue(new Shoot(m_shooter));
    ControllerConstants.operatorController.x().whileTrue(new Push(m_pusher));
    ControllerConstants.operatorController.y().toggleOnTrue(
      new SequentialCommandGroup(new Shoot(m_shooter).withTimeout(3),
        new ParallelCommandGroup(
          new Shoot(m_shooter),
          new Push(m_pusher)
        ).withTimeout(2)
      )
    );
   }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
