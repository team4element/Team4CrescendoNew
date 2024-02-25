// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commands.RollBack;
import frc.robot.Commands.RollBoth;
import frc.robot.Commands.Shoot;
//import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Subsystems.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Conveyor;
import frc.robot.Subsystems.Shooter;
//import frc.robot.Commands.OpenLoopDrive;


public class RobotContainer {
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  // Subsystems
  public static final CommandSwerveDrivetrain m_driveTrain = new CommandSwerveDrivetrain();
  public static final Conveyor m_conveyor = new Conveyor();

  private ArrayList<CANcoder> m_CANcoders;

 public static final Shooter m_shooter = new Shooter();
  // TODO: Should this go inside drivetrain class or should it be abstracted to RobotState class?
  private final Telemetry logger = new Telemetry(m_driveTrain.maxSpeedSupplier.get());


  public RobotContainer() {
    configureBindings();
    SmartDashboard.putData(m_chooser);
    m_driveTrain.registerTelemetry(logger::telemeterize);
    m_CANcoders = getSwerveCANcoder();
    configureBindings();
    m_driveTrain.setDefaultCommand(m_driveTrain.c_OpenLoopDrive());
  }

  private ArrayList<CANcoder> getSwerveCANcoder() {
    ArrayList<CANcoder> CANcoders = new ArrayList<CANcoder>();
    for(SwerveModule swerveModule : m_driveTrain.getSwerveModules()) {
      CANcoders.add(swerveModule.getCANcoder());
    }

    return CANcoders;

  }

  public Map<String, Double> getSwerveCANcoderPositions() {
    if (m_CANcoders == null) {
        getSwerveCANcoder();

    }

    Map<String, Double> CANcoderPositions = new HashMap<String, Double>();
    for(CANcoder temp_CANcoder : m_CANcoders) {
      CANcoderPositions.put(DriveTrainConstants.IDtoEncoderName.get(temp_CANcoder.getDeviceID()), temp_CANcoder.getPosition().getValue());
    }
    return CANcoderPositions;
  }

  private void configureBindings() {
    ControllerConstants.driveController.y().whileTrue(m_driveTrain.c_cardinalLock(0));
    ControllerConstants.driveController.x().whileTrue(m_driveTrain.c_cardinalLock(90));
    ControllerConstants.driveController.a().whileTrue(m_driveTrain.c_cardinalLock(180));
    ControllerConstants.driveController.b().whileTrue(m_driveTrain.c_cardinalLock(270));
    ControllerConstants.driveController.leftBumper().onTrue(m_driveTrain.c_seedFieldRelative());

    ControllerConstants.operatorController.y().whileTrue(m_conveyor.c_runBottom(Conveyor.Direction.INTAKE, 0.8));
    ControllerConstants.operatorController.x().whileTrue(m_conveyor.c_runTop(Conveyor.Direction.OUTTAKE, 0.8));

    ControllerConstants.operatorController.a().whileTrue(new RollBoth(m_conveyor));
    ControllerConstants.operatorController.leftBumper().whileTrue(new RollBack(m_conveyor));

    ControllerConstants.operatorController.b().whileTrue(m_shooter.c_runShooter(3000 / 60));



  }
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}
