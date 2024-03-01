package frc.robot.Subsystems;

import java.util.function.Supplier;
//import java.sql.Array;
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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.LiveDoubleBinding;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.TunerConstants;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {

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

    // Variables describing the robot's max speed
    LiveDoubleBinding maxSpeedBinding = new LiveDoubleBinding("Swerve", "maxSpeed", 3.0);
    LiveDoubleBinding maxAngularRateBinding = new LiveDoubleBinding("Swerve", "maxAngularRate", 1.5 * Math.PI);

    public Supplier<Double> maxSpeedSupplier = maxSpeedBinding.getSupplier();
    public Supplier<Double> maxAngularRateSupplier = maxAngularRateBinding.getSupplier();

    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();

    public final SwerveRequest.FieldCentricFacingAngle fieldCentricFacingAngle = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(DriveTrainConstants.kDeadZone)
            .withRotationalDeadband(DriveTrainConstants.kDeadZone)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    // Defines the type of driving when in open-loop (teleop)
    public final SwerveRequest.FieldCentric m_drive = new SwerveRequest.FieldCentric()
            .withDeadband(DriveTrainConstants.kDeadZone)
            .withRotationalDeadband(DriveTrainConstants.kDeadZone)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    // TODO: Need to tune
    // LiveDoubleBinding rotationPBinding = new LiveDoubleBinding("Swerve", "rotation/pBinding", 10.0, (event) -> {
    //     fieldCentricFacingAngle.HeadingController.setP(event.valueData.value.getDouble());
    // });

    // LiveDoubleBinding rotationIBinding = new LiveDoubleBinding("Swerve", "rotation/iBinding", 0.0, (event) -> {
    //     fieldCentricFacingAngle.HeadingController.setI(event.valueData.value.getDouble());
    // });

    // LiveDoubleBinding rotationDBinding = new LiveDoubleBinding("Swerve", "rotation/dBinding", 0.0, (event) -> {
    //     fieldCentricFacingAngle.HeadingController.setD(event.valueData.value.getDouble());
    // });

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
            SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        init();
    }

    public CommandSwerveDrivetrain() {
        super(TunerConstants.swerveConstants, m_driveFrontLeft, m_driveFrontRight, m_driveBackLeft, m_driveBackRight);
        init();
    }


    @Override
    public void periodic()
    {
        System.out.print("Gyro Data: ");
        System.out.println(m_pigeon2.getYaw()); // WE WANT 0 to be front
        System.out.println("Was Zero the front?");
    }


    private void configurePathPlanner() {
        double driveBaseRadius = 0;
        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }

        AutoBuilder.configureHolonomic(
                () -> this.getState().Pose, // Supplier of current robot pose
                this::seedFieldRelative,  // Consumer for seeding pose against auto
                this::getCurrentRobotChassisSpeeds,
                (speeds) -> this.setControl(autoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the robot
                new HolonomicPathFollowerConfig(
                        new PIDConstants(DriveTrainConstants.kTranslationP, DriveTrainConstants.kTranslationI, DriveTrainConstants.kTranslationD),
                        new PIDConstants(DriveTrainConstants.kRotationP, DriveTrainConstants.kRotationI, DriveTrainConstants.kRotationD),
                        TunerConstants.kSpeedAt12VoltsMps,
                        driveBaseRadius,
                        new ReplanningConfig()),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
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
        fieldCentricFacingAngle.HeadingController.setPID(DriveTrainConstants.kRotationP,
                DriveTrainConstants.kRotationI,
                DriveTrainConstants.kRotationD);
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
                                        * DriveTrainConstants.kSpeedMultiplyer) // Drive forward with negative Y (forward)
                        .withVelocityY(
                                ControllerConstants.xTranslationModifier
                                        .apply(-ControllerConstants.driveController.getLeftX())
                                        * DriveTrainConstants.kSpeedMultiplyer) // Drive left with negative X (left)
                        .withRotationalRate(
                                ControllerConstants.zRotationModifier
                                        .apply(-ControllerConstants.driveController.getRightX())
                                        * DriveTrainConstants.kSpeedMultiplyer) // Drive counterclockwise with negative X (left)
        );
    }

    public Command c_brake() {
        return applyRequest(() -> new SwerveRequest.SwerveDriveBrake());
    }

    public Command c_pointWheelsAt(double angle) {
        return applyRequest(() -> new SwerveRequest.PointWheelsAt().withModuleDirection(Rotation2d.fromDegrees(angle)));
    }

    public Command c_seedFieldRelative() {
        return runOnce(() -> seedFieldRelative());
    }
}