// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.OperatorConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.IMUSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Drive extends Command {
  /** Creates a new Drive. */
  CANDriveSubsystem driveSubsystem;
  CommandXboxController controller;

  IMUSubsystem imu = new IMUSubsystem();

  public Drive(CANDriveSubsystem driveSystem, CommandXboxController driverController, IMUSubsystem iimmuu) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSystem);
    driveSubsystem = driveSystem;
    controller = driverController;
    imu = iimmuu;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  // The Y axis of the controller is inverted so that pushing the
  // stick away from you (a negative value) drives the robot forwards (a positive
  // value). The X axis is scaled down so the rotation is more easily
  // controllable.
  @Override
  public void execute() {
    boolean isInverted = InvertDrive.getInvertedStatus();
    boolean OgDrive = ChangeDrive.isOriginalDrive();
    
    double trigger = controller.getRightTriggerAxis();

    double forwardScaling;
    double forward;
    double rotation;

    if (OgDrive) {
      forwardScaling = FEED_DRIVE_SCALING * (1.0 + trigger * (DRIVE_BOOST_FACTOR - 1.0));
      forward = controller.getLeftY() * forwardScaling;
      rotation = controller.getRightX() * FEED_ROTATION_SCALING;
    } else {
      forwardScaling = DEFENSE_DRIVE_SCALING * (1.0 + trigger * (DRIVE_BOOST_FACTOR - 1.0));
      forward = controller.getLeftY() * forwardScaling;
      rotation = controller.getRightX() * DEFENSE_ROTATION_SCALING;
    }
  
    // if (OgDrive) {
        if (isInverted) {
        driveSubsystem.driveArcade(forward, rotation);
      } else {
        driveSubsystem.driveArcade(-forward, rotation);
      }
    // } else {
    //   if (isInverted) {
    //     if (imu.getYaw() >= -90 && imu.getYaw() <= 90) {
    //       driveSubsystem.driveArcade(forward, rotation);
    //     } else if (imu.getYaw() < -90 || imu.getYaw() > 90) {
    //       driveSubsystem.driveArcade(-forward, -rotation);
    //     }
    //   } else {
    //     if (imu.getYaw() >= -90 && imu.getYaw() <= 90) {
    //       driveSubsystem.driveArcade(-forward, rotation);
    //     } else if (imu.getYaw() < -90 || imu.getYaw() > 90) {
    //       driveSubsystem.driveArcade(forward, -rotation);
    //     }
    //   }
    // }

    
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.driveArcade(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
