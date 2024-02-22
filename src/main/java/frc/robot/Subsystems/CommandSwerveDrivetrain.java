package frc.robot.Subsystems;

import java.util.function.Supplier;
import java.util.ArrayList;
import java.util.Arrays;

import com.ctre.phoenix6.Utils;
//import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.TunerConstants;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    // private double kP = 20;
    // private double kI = 0;
    // private double kD = 0;

    // Variables describing the robot's max speed
    public Supplier<Double> maxSpeedSupplier = () -> SmartDashboard.getNumber("maxSpeed", 3);
    public Supplier<Double> maxAngularRateSupplier = () -> SmartDashboard.getNumber("maxAngularRate", 1.5 * Math.PI);

    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();

    // private final SwerveRequest.SwerveDriveBrake brake = new
    // SwerveRequest.SwerveDriveBrake();
    // private final SwerveRequest.PointWheelsAt point = new
    // SwerveRequest.PointWheelsAt();

    public final SwerveRequest.FieldCentricFacingAngle fieldCentricFacingAngle = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(maxSpeedSupplier.get() * 0.1)
            .withRotationalDeadband(maxAngularRateSupplier.get() * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    // Defines the type of driving when in open-loop (teleop)
    public final SwerveRequest.FieldCentric m_drive = new SwerveRequest.FieldCentric()
            .withDeadband(maxSpeedSupplier.get() * 0.1)
            .withRotationalDeadband(maxAngularRateSupplier.get() * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
            SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        init();
    }

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        init();
    }

    private void configurePathPlanner() {
        double driveBaseRadius = 0;
        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }

        AutoBuilder.configureHolonomic(
                () -> this.getState().Pose, // Supplier of current robot pose
                this::seedFieldRelative, // Consumer for seeding pose against auto
                this::getCurrentRobotChassisSpeeds,
                (speeds) -> this.setControl(autoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the
                                                                             // robot
                new HolonomicPathFollowerConfig(new PIDConstants(10, 0, 0),
                        new PIDConstants(10, 0, 0),
                        TunerConstants.kSpeedAt12VoltsMps,
                        driveBaseRadius,
                        new ReplanningConfig()),
                () -> false, // Change this if the path needs to be flipped on red vs blue
                this); // Subsystem for requirements
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public Command getAutoPath(String pathName) {
        return new PathPlannerAuto(pathName);
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    private void init() {
        configurePathPlanner();
        // TODO: These need to be tuned to the real robot
        fieldCentricFacingAngle.HeadingController.setPID(10, 0, 0);
        fieldCentricFacingAngle.HeadingController.enableContinuousInput(-180, 180);

        // Each Subsystem has a default command that runs when no other command is
        setDefaultCommand(c_OpenLoopDrive());
        if (Utils.isSimulation()) {
            startSimThread();
            seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
        }
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    public ArrayList<SwerveModule> getSwerveModules() {
        return new ArrayList<>(Arrays.asList(this.Modules));
    }

    // Commands
    public Command c_cardinalLock(double angle) {
        return applyRequest( // could be better to change whileTrue to onTrue or toggleOnTrue
                () -> fieldCentricFacingAngle
                        .withVelocityX((-ControllerConstants.driveController.getLeftY()) * maxSpeedSupplier.get())
                        .withVelocityY((-ControllerConstants.driveController.getLeftX()) * maxSpeedSupplier.get())
                        .withTargetDirection(Rotation2d.fromDegrees(angle)));
    }

    public Command c_OpenLoopDrive() {
        return applyRequest(
                () -> m_drive
                        .withVelocityX(
                                ControllerConstants.yTranslationModifier
                                        .apply(-ControllerConstants.driveController.getLeftY())
                                        * maxSpeedSupplier.get()) // Drive forward with negative Y (forward)
                        .withVelocityY(
                                ControllerConstants.xTranslationModifier
                                        .apply(-ControllerConstants.driveController.getLeftX())
                                        * maxSpeedSupplier.get()) // Drive left with negative X (left)
                        .withRotationalRate(
                                ControllerConstants.zRotationModifier
                                        .apply(-ControllerConstants.driveController.getRightX())
                                        * maxAngularRateSupplier.get()) // Drive counterclockwise with negative X (left)
        );
    }

    public Command c_seedFieldRelative() {
        return runOnce(() -> seedFieldRelative());
    }
}