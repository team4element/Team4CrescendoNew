package frc.robot.Constants;

import java.util.HashMap;
import java.util.Map;

public class DriveTrainConstants{
    
    public static final int kFrontLeftDriveMotorId = 11;
    public static final int kFrontLeftSteerMotorId = 12;
    public static final int kFrontLeftEncoderId = 4;

    public static final int kFrontRightDriveMotorId = 9;
    public static final int kFrontRightSteerMotorId = 10;
    public static final int kFrontRightEncoderId = 2;

    public static final int kBackLeftDriveMotorId = 7;
    public static final int kBackLeftSteerMotorId = 8;
    public static final int kBackLeftEncoderId = 1;

    public static final int kBackRightDriveMotorId = 5;
    public static final int kBackRightSteerMotorId = 6;
    public static final int kBackRightEncoderId = 3;

    public static final double kTranslationP = 10.0;
    public static final double kTranslationI = 0.0;
    public static final double kTranslationD = 0.0;

    public static final double kRotationP = 10.0;
    public static final double kRotationI = 0.0;
    public static final double kRotationD = 0.0;

    public static final double kDeadZone = .8;
    public static final double kSpeedMultiplyer = 4;

    public static final Map<Integer, String> IDtoEncoderName = new HashMap<Integer, String>(); 

    DriveTrainConstants() {
        // intialize map values
        IDtoEncoderName.put(3, "FrontLeft");
        IDtoEncoderName.put(1, "FrontRight");
        IDtoEncoderName.put(2, "BackLeft");
        IDtoEncoderName.put(4, "BackRight");
    }
}