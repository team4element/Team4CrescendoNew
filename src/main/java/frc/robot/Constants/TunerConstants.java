package frc.robot.Constants;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
//import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;

//import edu.wpi.first.math.util.Units;

public class TunerConstants {
    // Both sets of gains need to be tuned to your individual robot.

    // The steer motor uses any SwerveModule.SteerRequestType control request with the
    // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
    public static final Slot0Configs steerGains = new Slot0Configs()
        .withKP(100).withKI(0).withKD(0.2)
        .withKS(0).withKV(1.5).withKA(0);
    // When using closed-loop control, the drive motor uses the control
    // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
    public static final Slot0Configs driveGains = new Slot0Configs()
        .withKP(3).withKI(0).withKD(0)
        .withKS(0).withKV(0).withKA(0);

    // The closed-loop output type to use for the steer motors;
    // This affects the PID/FF gains for the steer motors
    public static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
    // The closed-loop output type to use for the drive motors;
    // This affects the PID/FF gains for the drive motors
    public static final ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;

    // The stator current at which the wheels start to slip;
    // This needs to be tuned to your individual robot
    public static final double kSlipCurrentA = 300.0;

    // Theoretical free speed (m/s) at 12v applied output;
    // This needs to be tuned to your individual robot
    public static final double kSpeedAt12VoltsMps = 4.73;

    // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
    // This may need to be tuned to your individual robot
    public static final double kCoupleRatio = 3.5714285714285716;

    public static final double kDriveGearRatio = 6.746031746031747;
    public static final double kSteerGearRatio = 21.428571428571427;
    public static final double kWheelRadiusInches = 2;

    public static final boolean kSteerMotorReversed = true;
    public static final boolean kInvertLeftSide = false;
    public static final boolean kInvertRightSide = true;

    public static final String kCANbusName = "";
    public static final int kPigeonId = 0;


    // These are only used for simulation
    public static final double kSteerInertia = 0.00001;
    public static final double kDriveInertia = 0.001;
    // Simulated voltage necessary to overcome friction
    public static final double kSteerFrictionVoltage = 0.25;
    public static final double kDriveFrictionVoltage = 0.25;

    public static final SwerveDrivetrainConstants swerveConstants = new SwerveDrivetrainConstants()
            .withPigeon2Id(kPigeonId)
            .withCANbusName(kCANbusName);

    public static final SwerveModuleConstantsFactory ConstantCreator = new SwerveModuleConstantsFactory()
            .withDriveMotorGearRatio(kDriveGearRatio)
            .withSteerMotorGearRatio(kSteerGearRatio)
            .withWheelRadius(kWheelRadiusInches)
            .withSlipCurrent(kSlipCurrentA)
            .withSteerMotorGains(steerGains)
            .withDriveMotorGains(driveGains)
            .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
            .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
            .withSpeedAt12VoltsMps(kSpeedAt12VoltsMps)
            .withSteerInertia(kSteerInertia)
            .withDriveInertia(kDriveInertia)
            .withSteerFrictionVoltage(kSteerFrictionVoltage)
            .withDriveFrictionVoltage(kDriveFrictionVoltage)
            .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
            .withCouplingGearRatio(kCoupleRatio)
            .withSteerMotorInverted(kSteerMotorReversed);


    // Front Left
    public static final double kFrontLeftEncoderOffset = -0.185546875;

    public static final double kFrontLeftXPosInches = 11.5;
    public static final double kFrontLeftYPosInches = 13.5;

    // Front Right
    public static final double kFrontRightEncoderOffset = -0.4326171875;
   

    public static final double kFrontRightXPosInches = 11.5;
    public static final double kFrontRightYPosInches = -13.5;

    // Back Left
    public static final double kBackLeftEncoderOffset = -0.090576171875;


    public static final double kBackLeftXPosInches = -11.5;
    public static final double kBackLeftYPosInches = 13.5;

    // Back Right
    public static final double kBackRightEncoderOffset = 0.177001953125;
  

    public static final double kBackRightXPosInches = -11.5;
    public static final double kBackRightYPosInches = -13.5;



}
