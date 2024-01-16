// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;

/**
 * Question:
 * 1. How does the robot know what orientation the field is?
 * 2.
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
  private final Telemetry logger = new Telemetry(MaxSpeed);

  private void configureBindings() {
    // The default "drive" command when no other commands are using the drivetrain.
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(
            () -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));

    joystick.b().whileTrue(drivetrain.applyRequest(
      () -> point.withModuleDirection(
        new Rotation2d(
          -joystick.getLeftY(),
          -joystick.getLeftX()
        )
      )
    ));

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
    return Commands.print("No autonomous command configured");
  }
}
