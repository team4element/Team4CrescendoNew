// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*package frc.robot.Commands.Autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ElementUnits;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.TunerConstants;
import frc.robot.Subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public class ProfiledDrivetoPosition extends Command {
  
  ProfiledPIDController positionPID;

  CommandSwerveDrivetrain m_driveTrain;
  double m_position;

  double tolerance = ElementUnits.inchesToTicks(0.1);

  
  public ProfiledDrivetoPosition(CommandSwerveDrivetrain drive, double position,double maxAcceleration) {
 
    this.m_driveTrain = drive;
    this.m_position = position; 

    positionPID = new ProfiledPIDController(maxAcceleration, maxAcceleration, maxAcceleration, null);
  }

 
  @Override
  public void initialize() {}

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
*/
