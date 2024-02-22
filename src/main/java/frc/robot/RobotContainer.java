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
//import com.pathplanner.lib.path.PathConstraints;
//import com.pathplanner.lib.path.PathPlannerTrajectory;
//import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.OpenLoopDrive;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.TunerConstants;
import frc.robot.Subsystems.CommandSwerveDrivetrain;

//import frc.robot.Subsystems.Conveyor;
//import frc.robot.Subsystems.Pusher;
//import frc.robot.Subsystems.Shooter;
//import frc.robot.Commands.RollBack;
//import frc.robot.Commands.Shoot;
//import frc.robot.Commands.ShootBack;
//import frc.robot.Commands.Take;
//import frc.robot.Commands.Roll;
//import frc.robot.Commands.Leave;
//import frc.robot.Commands.Push;
/**
 * What does Auton Path Following Mean?
 * 
 * Drives on pre-written code.
 * There is going to be a Command called "FollowDrivePath".
 * 
 * This command will need a predetermined path to follow.
 * 
 * How are commands scheduled/run?
 * 1. In Teleop, When a "trigger" is fired, it tells the Scheduler to run a
 * command.
 * 2. A subsystem can be given a "Default command". This is scheduled when NO
 * OTHER COMMANDS are using the subsystem.
 * 
 * 
 * In Teleop, commands are scheduled to run when a certain "trigger" is
 * triggered.
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

  // TODO: Sendable Chooser for auton
  // 1:PathPlannerTrajectory driveForward = PathPlannerTrajectory("Drive Forward
  // from the right", new PathConstraints(3,3, 0, 0));

  // 2: ArrayList<PathPlannerTrajectory> pathGroup = PathPlannerTrajectory("Drive
  // on left", new PathConstraints(3,3, 0, 0));

  // HashMap<String, Command> eventMap = new HashMap<>();
  // eventMap("drive", new PrintCommand("go forward"));

  // SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
  // m_driveTrain::getPose,
  // m_driveTrain::resetPose,
  // m_driveTrain.kinematics,
  // new PIDConstants(0, 0, 0),
  // new PIDConstants(0,0,0),
  // m_driveTrain::setModuleStates,
  // eventMap,
  // true,
  // m_driveTrain

  // );

  // Command fullAuto = autoBuilder.fullAuto(pathGroup);

  // 3. public static final Command m_auto =
  // new DriveDistance(
  // AutonConstants.kAutonDriveDistanceInches,DriveTrainConstants.kAutoDriveSpeed,
  // m_driveTrain);

  // public final Command m_driveForward = new DriveForward (m_driveTrain,
  // m_shooter);

  SendableChooser<Command> m_chooser = new SendableChooser<>();

  // Subsystems

  // Swerve Drive (This is like our entire driveTrain)
  public static final CommandSwerveDrivetrain m_driveTrain = new CommandSwerveDrivetrain(TunerConstants.swerveConstants,
      m_driveFrontLeft,
      m_driveFrontRight, m_driveBackLeft, m_driveBackRight);
  // Conveyor
  // public static final Conveyor m_conveyor = new Conveyor();
  // Shooter
  // public static final Shooter m_shooter = new Shooter();
  // Pusher
  // public static final Pusher m_pusher = new Pusher();

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


      // ControllerConstants.operatorController.a().onTrue(
      // new Take(m_intake)
      // );

      // ControllerConstants.operatorController.y().onTrue(
      // new Leave(m_intake)
      // );

      // ControllerConstants.operatorController.x().onTrue(
      // new Push(m_pusher)
      // );


    ControllerConstants.driveController.leftBumper().onTrue(
        m_driveTrain.runOnce(
            () -> m_driveTrain.seedFieldRelative()));

    // ControllerConstants.operatorController.leftBumper().onTrue(
    // new RollBack(m_conveyor)

    // );

    // ControllerConstants.operatorController.leftTrigger().onTrue(
    // new Roll(m_conveyor)
    // );

    // ControllerConstants.operatorController.rightTrigger().onTrue(
    // new Shoot(m_shooter)
    // );

    // ControllerConstants.operatorController.rightBumper().onTrue(
    // new ShootBack(m_shooter)
    // );

    if (Utils.isSimulation()) {
      m_driveTrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    m_driveTrain.registerTelemetry(logger::telemeterize);
  }

  /// private ArrayList<com.pathplanner.lib.path.PathPlannerTrajectory>
  /// PathPlannerTrajectory(String string, PathConstraints pathConstraints) {
  // TODO Auto-generated method stub
  // throw new UnsupportedOperationException("Unimplemented method
  /// 'PathPlannerTrajectory'");
  // }

  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
    // TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
    // 0, 0)
    // .setKinematics(CommandSwerveDrivetrain.m_kinematics);
  }

  /**
   * Sets the default command per subsystem
   */
  private void setDefaultCommands() {
    // Add more default commands here
    // m_driveTrain.setDefaultCommand(new OpenLoopDrive(m_driveTrain).GetCommand());
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
