// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANDriveSubsystem;
import static frc.robot.Constants.AimAndRangeConstants.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoDrive extends Command {
  /** Creates a new Drive. */
  CANDriveSubsystem driveSubsystem;
  double targetDistance, rotation;

  // private final PIDController drivePID = new PIDController(DRIVE_P, DRIVE_I, DRIVE_D);

  // private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(DRIVE_kS, DRIVE_kV, DRIVE_kA);

  public AutoDrive(CANDriveSubsystem driveSystem, double distance, double r) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSystem);
    driveSubsystem = driveSystem;
    this.targetDistance = distance;
    this.rotation = r;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // drivePID.reset();
    // // reset encoders to start measuring distance from zero
    // driveSubsystem.resetEncoders();
    // // set a reasonable tolerance for completion (meters)
    // drivePID.setTolerance(0.02); // 2 cm
  }

  // Called every time the scheduler runs while the command is scheduled.
  // Setting the values here instead of in initialize feeds the watchdog on the
  // arcade drive object
  @Override
  public void execute() {
  // // current measured distance (meters)
  // double currentDistance = driveSubsystem.getAverageDistanceMeters();
  // double pidDrive = drivePID.calculate(currentDistance, targetDistance);

  // // --- Drive feedforward ---
  // double errorDist = targetDistance - currentDistance;
  // double desiredLinVel = Math.max(-DRIVE_MAX_VEL_M_PER_S,
  //   Math.min(DRIVE_MAX_VEL_M_PER_S, errorDist * DRIVE_VEL_SCALE));
  //   double ffDriveVolts = driveFeedforward.calculate(desiredLinVel);
  //   double ffDrive = ffDriveVolts / Math.max(0.001, RobotController.getBatteryVoltage());

  //   double forwardSpeed = pidDrive + ffDrive;
  //   forwardSpeed  = Math.max(-1.0, Math.min(1.0, forwardSpeed));

    driveSubsystem.driveArcade(targetDistance, rotation);
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
