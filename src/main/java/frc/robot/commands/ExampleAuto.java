// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.CANFuelSubsystem;

public class ExampleAuto extends SequentialCommandGroup {
  /** Creates a new ExampleAuto. */
  public ExampleAuto(SwerveDriveSubsystem driveSubsystem, CANFuelSubsystem ballSubsystem) {
    addCommands(
      // Drive forward for 0.25 seconds using swerve ChassisSpeeds
      new FunctionalCommand(
        () -> {},
        () -> driveSubsystem.drive(new ChassisSpeeds(1.0, 0.0, 0.0)),
        (interrupted) -> driveSubsystem.drive(new ChassisSpeeds()),
        () -> false,
        driveSubsystem
      ).withTimeout(0.25),
      // Spin up and launch for 10 seconds
      new Launch(ballSubsystem).withTimeout(10)
    );
  }
}
