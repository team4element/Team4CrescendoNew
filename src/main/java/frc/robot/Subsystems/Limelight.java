// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.Subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class Limelight extends SubsystemBase {

  private NetworkTable m_limelightNT;
  private NetworkTableEntry m_tx; 
  private NetworkTableEntry m_ty;
  private NetworkTableEntry m_ta;

  private double m_x;
  private double m_y;
  private double m_area;
 
  public Limelight() {
    m_limelightNT = NetworkTableInstance.getDefault().getTable("limelight");
    m_tx = m_limelightNT.getEntry("tx");
    m_ty = m_limelightNT.getEntry("ty");
    m_ta = m_limelightNT.getEntry("ta");
  }


  /**
   * Gets the distance of the target from the Limelight in inches
   * @return Distance in inches 
   */
  public double getDistance()
  {
    return (VisionConstants.goalHeightInches - VisionConstants.limelightLensHeightInches) / Math.tan(getAngleToGoalRadians());
  }

  /**
   * Gets the angle of the of the target from the Limelight
   * @return Angle in degrees
   */
  public double getAngleToTargetDegrees()
  {
    return VisionConstants.limelightMountAngleDegrees + m_y;
  }

  /**
   * Gets the angle to the goal in radians
   * @return Angle in radians
   */
  public double getAngleToGoalRadians()
  {
    return getAngleToTargetDegrees() * (Math.PI/180.0);
  }

  @Override
  public void periodic() {
    m_x    = m_tx.getDouble(0.0);
    m_y    = m_ty.getDouble(0.0);
    m_area = m_ta.getDouble(0.0);

    SmartDashboard.putNumber("LimeLightX"   , m_x);
    SmartDashboard.putNumber("LimeLightY"   , m_y);
    SmartDashboard.putNumber("LimeLightArea", m_area);

  }
}
