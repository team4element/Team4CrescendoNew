// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.TunerConstants;
import frc.robot.Subsystems.CommandSwerveDrivetrain;
import frc.robot.Subsystems.Conveyor;
import frc.robot.Commands.OpenLoopDrive;

public class RobotContainer {
  private static final SwerveModuleConstants m_driveFrontLeft = TunerConstants.ConstantCreator.createModuleConstants(
      DriveTrainConstants.kFrontLeftSteerMotorId,
      DriveTrainConstants.kFrontLeftDriveMotorId,
      DriveTrainConstants.kFrontLeftEncoderId,
      TunerConstants.kFrontLeftEncoderOffset,
      Units.inchesToMeters(TunerConstants.kFrontLeftXPosInches),
      Units.inchesToMeters(TunerConstants.kFrontLeftYPosInches),
      TunerConstants.kInvertLeftSide);

  private static final SwerveModuleConstants m_driveFrontRight = TunerConstants.ConstantCreator.createModuleConstants(
      DriveTrainConstants.kFrontRightSteerMotorId,
      DriveTrainConstants.kFrontRightDriveMotorId,
      DriveTrainConstants.kFrontRightEncoderId,
      TunerConstants.kFrontRightEncoderOffset,
      Units.inchesToMeters(TunerConstants.kFrontRightXPosInches),
      Units.inchesToMeters(TunerConstants.kFrontRightYPosInches),
      TunerConstants.kInvertRightSide);

  private static final SwerveModuleConstants m_driveBackLeft = TunerConstants.ConstantCreator.createModuleConstants(
      DriveTrainConstants.kBackLeftSteerMotorId,
      DriveTrainConstants.kBackLeftDriveMotorId,
      DriveTrainConstants.kBackLeftEncoderId,
      TunerConstants.kBackLeftEncoderOffset,
      Units.inchesToMeters(TunerConstants.kBackLeftXPosInches),
      Units.inchesToMeters(TunerConstants.kBackLeftYPosInches),
      TunerConstants.kInvertLeftSide);

  private static final SwerveModuleConstants m_driveBackRight = TunerConstants.ConstantCreator.createModuleConstants(
      DriveTrainConstants.kBackRightSteerMotorId,
      DriveTrainConstants.kBackRightDriveMotorId,
      DriveTrainConstants.kBackRightEncoderId,
      TunerConstants.kBackRightEncoderOffset,
      Units.inchesToMeters(TunerConstants.kBackRightXPosInches),
      Units.inchesToMeters(TunerConstants.kBackRightYPosInches),
      TunerConstants.kInvertRightSide);

  SendableChooser<Command> m_chooser = new SendableChooser<>();

  // Subsystems
  public static final CommandSwerveDrivetrain m_driveTrain = new CommandSwerveDrivetrain(TunerConstants.swerveConstants,
      m_driveFrontLeft,
      m_driveFrontRight, m_driveBackLeft, m_driveBackRight);
  public static final Conveyor m_conveyor = new Conveyor();

  private final Telemetry logger = new Telemetry(m_driveTrain.maxSpeedSupplier.get());

  private ArrayList<CANcoder> m_CANcoders;

  public RobotContainer() {
    configureBindings();
    SmartDashboard.putData(m_chooser);

    configureBindings();
    setDefaultCommands();
    m_CANcoders = getSwerveCANcoders();
  }

  private void configureBindings() {
    ControllerConstants.driveController.y().whileTrue(m_driveTrain.c_cardinalLock(0));
    ControllerConstants.driveController.x().whileTrue(m_driveTrain.c_cardinalLock(90));
    ControllerConstants.driveController.a().whileTrue(m_driveTrain.c_cardinalLock(180));
    ControllerConstants.driveController.b().whileTrue(m_driveTrain.c_cardinalLock(270));
    ControllerConstants.driveController.leftBumper().onTrue(m_driveTrain.c_seedFieldRelative());

    ControllerConstants.operatorController.y().whileTrue(m_conveyor.c_runBottom(Conveyor.State.INTAKE, 0.5));
    ControllerConstants.operatorController.x().whileTrue(m_conveyor.c_runTop(Conveyor.State.OUTTAKE, 0.5));

    ControllerConstants.operatorController.a().whileTrue(Commands.parallel(
        m_conveyor.c_runBottom(Conveyor.State.INTAKE, 0.5),
        m_conveyor.c_runTop(Conveyor.State.INTAKE, 0.5)));

    ControllerConstants.operatorController.b().whileTrue(Commands.parallel(
        m_conveyor.c_runBottom(Conveyor.State.OUTTAKE, 0.5),
        m_conveyor.c_runTop(Conveyor.State.OUTTAKE, 0.5)));

    if (Utils.isSimulation()) {
      m_driveTrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    m_driveTrain.registerTelemetry(logger::telemeterize);
  }

  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }

  private void setDefaultCommands() {
    m_driveTrain.setDefaultCommand(new OpenLoopDrive(m_driveTrain).GetCommand());
  }

  private ArrayList<CANcoder> getSwerveCANcoders() {
    ArrayList<CANcoder> CANcoders = new ArrayList<CANcoder>();
    for (SwerveModule swerveModule : m_driveTrain.getSwerveModules()) {
      CANcoders.add(swerveModule.getCANcoder());
    }
    return CANcoders;
  }

  public Map<String, Double> getSwerveCANcoderPositions() {
    if (m_CANcoders == null) {
      getSwerveCANcoders();
    }
    Map<String, Double> CANcoderPositions = new HashMap<String, Double>();
    for (CANcoder temp_CANcoder : m_CANcoders) {
      CANcoderPositions.put(DriveTrainConstants.IDtoEncoderName.get(temp_CANcoder.getDeviceID()),
          temp_CANcoder.getPosition().getValue());
    }
    return CANcoderPositions;
  }
}
