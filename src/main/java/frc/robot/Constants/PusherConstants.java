package frc.robot.Constants;

public class PusherConstants {
  
    public static final int motorId = 16;

    public static final double lowSpeed = .1;
    public static final double medSpeed = .5;
    public static final double highSpeed = .08;

    public static final double kP = .000555;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final int potID = 0;
    public static final int potLimitHigh = 900;
    public static final int potLimitLow  = 100;
    public static final int potMax = 1024;

    public static final int encoderPosition = 5128;

    public static final double kGearRatio = 10/1;

    public static final double kPulsePerRev = 4090;

    public static final double kRed1 = 0.561;
    public static final double kRed2 = 0.232;
    public static final double kRed3 = 0.114;

    public static final double encoderOffset = 0.402099609375;

    public static final String tableP = "Pusher P";
    public static final String tableI = "Pusher I";
    public static final String tableD = "Pusher I";
}