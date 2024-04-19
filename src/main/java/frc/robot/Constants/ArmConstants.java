// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;

/** Add your docs here. */
public class ArmConstants {
//TODO: change ids later
    public static final int m_angleMotorID = 20;
    public static final int m_shootMotorID = 19;

    public static final int angleCurrentLimitAmps = 80;
    public static final int shootCurrentLimitAmps = 80;

    public static final double forwardLimit = 100;
    public static final double reverseLimit = -100;

    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;

    public static double shootSpeed = .6;

    public static final String tableP = "Arm P";
    public static final String tableI = "Arm I";
    public static final String tableD = "Arm D";
}
