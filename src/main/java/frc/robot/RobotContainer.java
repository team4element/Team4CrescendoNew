// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Commands.Shoot;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Subsystems.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Conveyor;
import frc.robot.Subsystems.Pusher;
//import frc.robot.Commands.OpenLoopDrive;
import frc.robot.Commands.ShootBack;
import frc.robot.Subsystems.Shooter;

public class RobotContainer {
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  // Subsystems
  public static final CommandSwerveDrivetrain m_driveTrain = new CommandSwerveDrivetrain();
  public static final Conveyor m_conveyor = new Conveyor();
  public static final Shooter m_shooter = new Shooter();
  public static final Pusher m_pusher = new Pusher();
  // TODO: Should this go inside drivetrain class or should it be abstracted to RobotState class?
  private final Telemetry logger = new Telemetry(m_driveTrain.maxSpeedSupplier.get());
  

  public RobotContainer() {
    configureBindings();
    SmartDashboard.putData(m_chooser);
    m_driveTrain.registerTelemetry(logger::telemeterize);

    configureBindings();
  }

  private void configureBindings() {
    ControllerConstants.driveController.y().whileTrue(m_driveTrain.c_cardinalLock(0));
    ControllerConstants.driveController.x().whileTrue(m_driveTrain.c_cardinalLock(90));
    ControllerConstants.driveController.a().whileTrue(m_driveTrain.c_cardinalLock(180));
    ControllerConstants.driveController.b().whileTrue(m_driveTrain.c_cardinalLock(270));
    ControllerConstants.driveController.leftBumper().onTrue(m_driveTrain.c_seedFieldRelative());

    ControllerConstants.operatorController.y().whileTrue(m_conveyor.c_runBottom(Conveyor.Direction.INTAKE, 0.5));
    ControllerConstants.operatorController.x().whileTrue(m_conveyor.c_runTop(Conveyor.Direction.OUTTAKE, 0.5));

    ControllerConstants.operatorController.a().whileTrue(m_conveyor.c_runBoth(Conveyor.Direction.INTAKE, 0.5));
    ControllerConstants.operatorController.b().whileTrue(m_conveyor.c_runBoth(Conveyor.Direction.OUTTAKE, 0.5));

    ControllerConstants.operatorController.rightTrigger().whileTrue(
      new Shoot(m_shooter)

    );

    ControllerConstants.operatorController.rightBumper().whileTrue(
      new ShootBack(m_shooter)

    );

  }
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}
