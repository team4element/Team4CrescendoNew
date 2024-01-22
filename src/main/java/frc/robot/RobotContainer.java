// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.fasterxml.jackson.annotation.JsonAnyGetter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;

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
  // Variables describing the robot's max speed
  private double MaxSpeed = 6; // meters per second
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  // Setting up joystick to attach triggers later
  private final CommandXboxController joystick = new CommandXboxController(0);

  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;

  // Defines the type of driving when in open-loop (teleop)
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1)
      .withRotationalDeadband(MaxAngularRate * 0.1)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final SwerveRequest.FieldCentricFacingAngle fieldCentricFacingAngle = new SwerveRequest.FieldCentricFacingAngle()
  .withDeadband(MaxSpeed * 0.1)
  .withRotationalDeadband(MaxAngularRate * 0.1)
  .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private void configureBindings() {
    // The default "drive" command when no other commands are using the drivetrain.
    JoystickModifier yTranslationModifier = new JoystickModifier("yTranslationModifier");
    JoystickModifier xTranslationModifier = new JoystickModifier("xTranslationModifier");
    JoystickModifier zRotationModifier = new JoystickModifier("zRotationModifier");

    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(
            () -> drive.withVelocityX(yTranslationModifier.apply(-joystick.getLeftY()) * MaxSpeed) // Drive forward with negative Y (forward)
                .withVelocityY(xTranslationModifier.apply(-joystick.getLeftX()) * MaxSpeed) // Drive left with negative X (left)
                .withRotationalRate(zRotationModifier.apply(-joystick.getRightX()) * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

    // TODO: These need to be tuned to the real robot
    fieldCentricFacingAngle.HeadingController.setPID(10,0,0);
    fieldCentricFacingAngle.HeadingController.enableContinuousInput(-Math.PI, Math.PI);


    Trigger[] cardinalDirectionLockTriggers = { joystick.y(), joystick.x(), joystick.a(), joystick.b() };
    // Loop through these triggers, and add 90 degrees to each one

    for (int i = 0; i < cardinalDirectionLockTriggers.length; i++) {
      Trigger trigger = cardinalDirectionLockTriggers[i];
      final int finalPosition = i;

      trigger.whileTrue(drivetrain.applyRequest(
        () -> fieldCentricFacingAngle
        .withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
        .withVelocityY((-joystick.getLeftX()) * MaxSpeed) // Drive left with negative X (left)
        .withTargetDirection(Rotation2d.fromDegrees(finalPosition * 90))));
    }

    joystick.leftBumper().onTrue(
      drivetrain.runOnce(
        () -> drivetrain.seedFieldRelative()
      )
    );

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public RobotContainer() {
    configureBindings();
  }

  public Command getAutonomousCommand() {
    return drivetrain.getAutoPath("Tests");
  }
}
