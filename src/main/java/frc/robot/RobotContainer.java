// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
//import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
//import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.OpenLoopDrive;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.TunerConstants;
import frc.robot.Subsystems.CommandSwerveDrivetrain;


/**
 * What does Auton Path Following Mean?
 * 
 * Drives on pre-written code.
 * There is going to be a Command called "FollowDrivePath".
 * 
 * This command will need a predetermined path to follow.
 * 
 * How are commands scheduled/run?
 * 1. In Teleop, When a "trigger" is fired, it tells the Scheduler to run a command.
 * 2. A subsystem can be given a "Default command". This is scheduled when NO OTHER COMMANDS are using the subsystem.
 * 
 * 
 * In Teleop, commands are scheduled to run when a certain "trigger" is triggered.
 * 
 * 
 * 
 * 
 */
public class RobotContainer {

  // Setting up joystick to attach triggers later

  
  // Swerve Modules (think of this as a wheel)
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
  
  //Swerve Drive (This is like our entire driveTrain)
  public static final CommandSwerveDrivetrain m_driveTrain = new CommandSwerveDrivetrain(TunerConstants.swerveConstants, m_driveFrontLeft,
  m_driveFrontRight, m_driveBackLeft, m_driveBackRight);

  private final Telemetry logger = new Telemetry(m_driveTrain.maxSpeedSupplier.get());

  private ArrayList<CANcoder> m_CANcoders; 

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

      // TODO: move to command not sure what this does here. Does this move the turn motors?
      trigger.whileTrue(m_driveTrain.applyRequest( // could be better to change whileTrue to onTrue or toggleOnTrue
        () -> m_driveTrain.fieldCentricFacingAngle
        .withVelocityX( -ControllerConstants.driveController.getLeftY() * m_driveTrain.maxSpeedSupplier.get()) // Drive forward with negative Y (forward)
        .withVelocityY((-ControllerConstants.driveController.getLeftX()) * m_driveTrain.maxSpeedSupplier.get()) // Drive left with negative X (left)
        .withTargetDirection(Rotation2d.fromDegrees(finalPosition * 90))));
    }

    ControllerConstants.driveController.leftBumper().onTrue(
      m_driveTrain.runOnce(
        () -> m_driveTrain.seedFieldRelative()
      )
    );

    if (Utils.isSimulation()) {
      m_driveTrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    m_driveTrain.registerTelemetry(logger::telemeterize);
  }

  public RobotContainer() {
    configureBindings();
    setDefaultCommands();
    m_CANcoders = getSwerveCANcoders();
  }

  public Command getAutonomousCommand() {
    return m_driveTrain.getAutoPath("Tests");
  }

  /**
   * Sets the default command per subsystem
   */
  private void setDefaultCommands()
  {
    //Add more default commands here
    m_driveTrain.setDefaultCommand(new OpenLoopDrive(m_driveTrain).GetCommand());
  }

  private ArrayList<CANcoder> getSwerveCANcoders() {
    ArrayList<CANcoder> CANcoders = new ArrayList<CANcoder>();
    for(SwerveModule swerveModule : m_driveTrain.getSwerveModules()) {
      CANcoders.add(swerveModule.getCANcoder());
    }
    return CANcoders;
  }

  public Map<String, Double> getSwerveCANcoderPositions() {
    if (m_CANcoders == null) {
      getSwerveCANcoders();
    }
    Map<String, Double> CANcoderPositions = new HashMap<String, Double>();
    for(CANcoder temp_CANcoder : m_CANcoders) {
      CANcoderPositions.put(DriveTrainConstants.IDtoEncoderName.get(temp_CANcoder.getDeviceID()), temp_CANcoder.getPosition().getValue());
    }
    return CANcoderPositions;
  }
}
