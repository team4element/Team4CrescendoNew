// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.Commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.Subsystems.Pusher;
// import frc.robot.Subsystems.Shooter;

// public class PushShoot extends Command {
//   Pusher m_pusher = new Pusher();
//   Shooter m_shooter = new Shooter();

//   public PushShoot(Pusher pusher, Shooter shooter) {
//     m_pusher = pusher;
//     m_shooter = shooter;

//   }


//   @Override
//   public void initialize() {}


//   @Override
//   public void execute() {

//     new SequentialCommandGroup(m_shooter.c_runShooter(0.5).withTimeout(3).andThen(new ParallelCommandGroup(m_shooter.c_runShooter(0.5), m_pusher.c_runPusher(0.5))));

//   }



//   @Override
//   public void end(boolean interrupted) {

//     m_shooter.motorsOff();
//     m_pusher.motorsOff();

//   }

//     @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
