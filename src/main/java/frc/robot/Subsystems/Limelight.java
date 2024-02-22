// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
/* 
package frc.robot.Subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class Limelight extends SubsystemBase {

  NetworkTable limelightNT = NetworkTableInstance.getDefault().getTable("Limelight");

  double getAngle;
  double getDistance;
 
  public Limelight() {}

  @Override
  public void periodic() {
    double getX = limelightNT.getEntry("x").getDouble(0);
    double getY = limelightNT.getEntry("y").getDouble(0);

    getDistance = (VisionConstants.kTargetHeight - VisionConstants.kFloorToLens) /
      Math.tan(Math.toRadians(VisionConstants.kFloorToLensAngle + getY));

    getAngle = getX;
   
  }
}
*/
