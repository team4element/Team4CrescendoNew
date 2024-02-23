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
import frc.robot.Subsystems.Pusher;
import frc.robot.Subsystems.Shooter;
import frc.robot.Commands.OpenLoopDrive;
import frc.robot.Commands.Shoot;
import frc.robot.Commands.ShootBack;

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
  public static final Shooter m_shooter = new Shooter();

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
    Trigger[] cardinalDirectionLockTriggers = {
        ControllerConstants.driveController.y(),
        ControllerConstants.driveController.x(),
        ControllerConstants.driveController.a(),
        ControllerConstants.driveController.b() };
    // Loop through these triggers, and add 90 degrees to each one
    for (int i = 0; i < cardinalDirectionLockTriggers.length; i++) {
      Trigger trigger = cardinalDirectionLockTriggers[i];
      final int finalPosition = i;

      trigger.whileTrue(m_driveTrain.applyRequest( // could be better to change whileTrue to onTrue or toggleOnTrue
          () -> m_driveTrain.fieldCentricFacingAngle
              .withVelocityX((-ControllerConstants.driveController.getLeftY()) * m_driveTrain.maxSpeedSupplier.get())
              .withVelocityY((-ControllerConstants.driveController.getLeftX()) * m_driveTrain.maxSpeedSupplier.get())
              .withTargetDirection(Rotation2d.fromDegrees(finalPosition * 90))));
    }

    ControllerConstants.driveController.leftBumper().onTrue(
        m_driveTrain.runOnce(
            () -> m_driveTrain.seedFieldRelative()));

    // Conveyor
    Command intakeBottom = Commands.startEnd(
        () -> m_conveyor.setBottom(-0.5),
        () -> m_conveyor.setBottom(0),
        m_conveyor);

    Command intakeTop = Commands.startEnd(
        () -> m_conveyor.setTop(-0.5),
        () -> m_conveyor.setTop(0),
        m_conveyor);

    Command outtakeBottom = Commands.startEnd(
        () -> m_conveyor.setBottom(0.5),
        () -> m_conveyor.setBottom(0),
        m_conveyor);

    Command outtakeTop = Commands.startEnd(
        () -> m_conveyor.setTop(0.5),
        () -> m_conveyor.setTop(0),
        m_conveyor);

        //Trigger
    ControllerConstants.operatorController.y().whileTrue(intakeBottom);
    ControllerConstants.operatorController.x().whileTrue(intakeTop);
   /*  ControllerConstants.operatorController.a().whileTrue(Commands.parallel(intakeBottom, intakeTop));
    ControllerConstants.operatorController.b().whileTrue(Commands.parallel(outtakeBottom, outtakeTop));


      //Shooter

    ControllerConstants.operatorController.rightTrigger().whileTrue(
      new Shoot(m_shooter)
      
    );

    ControllerConstants.operatorController.rightBumper().whileTrue(
      new ShootBack(m_shooter)
      
   );  
*/
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
