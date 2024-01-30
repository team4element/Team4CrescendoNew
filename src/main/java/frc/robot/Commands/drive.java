// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Subsystems.CommandSwerveDrivetrain;

public class drive extends Command {
  private CommandSwerveDrivetrain m_driveTrain;

  public drive(CommandSwerveDrivetrain driveTrain) {
    m_driveTrain = driveTrain;

    addRequirements(m_driveTrain);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      m_driveTrain.applyRequest(
      () -> m_driveTrain.m_drive
          .withVelocityX(ControllerConstants.yTranslationModifier.apply  (-ControllerConstants.driveController.getLeftY())  * m_driveTrain.maxSpeedSupplier.get()) // Drive forward with negative Y (forward)
          .withVelocityY(ControllerConstants.xTranslationModifier.apply  (-ControllerConstants.driveController.getLeftX())  * m_driveTrain.maxSpeedSupplier.get()) // Drive left with negative X (left)
          .withRotationalRate(ControllerConstants.zRotationModifier.apply(-ControllerConstants.driveController.getRightX()) * m_driveTrain.maxAngularRateSupplier.get()) // Drive counterclockwise with negative X (left)
      );

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
