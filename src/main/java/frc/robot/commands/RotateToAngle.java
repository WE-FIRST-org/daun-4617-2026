// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.IMUSubsystem;

import static frc.robot.Constants.AimAndRangeConstants.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RotateToAngle extends Command {
  /** Creates a new Drive. */
  CANDriveSubsystem driveSubsystem;
  IMUSubsystem imuSubsystem;

  double currAngle, targetAngle;
  // angle offset in degrees relative to current heading; if you want absolute, pass offset from 0
  double angleOffsetDeg;

  private final PIDController turnPID = new PIDController(TURN_P, TURN_I,TURN_D);
  // private final SimpleMotorFeedforward turnFeedforward = new SimpleMotorFeedforward(TURN_kS, TURN_kV, TURN_kA);


  public RotateToAngle(CANDriveSubsystem driveSystem, IMUSubsystem imuSystem, double angleOffsetDeg) {
    // Require the drive subsystem only
    addRequirements(driveSystem);
    this.driveSubsystem = driveSystem;
    this.imuSubsystem = imuSystem;
    this.angleOffsetDeg = angleOffsetDeg;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turnPID.reset();
    // compute target as current yaw + offset (keeps target relative to heading at press time)
    currAngle = imuSubsystem.getYaw();
    targetAngle = currAngle + angleOffsetDeg;
    // configure tolerances (degrees)
    turnPID.setTolerance(2.0); // 2 degrees tolerance
  }

  // Called every time the scheduler runs while the command is scheduled.
  // Setting the values here instead of in initialize feeds the watchdog on the
  // arcade drive object
  @Override
  public void execute() {
    // read live angle
    currAngle = imuSubsystem.getYaw();
    double pidOutput = turnPID.calculate(currAngle, targetAngle);

    // --- FEEDFORWARD SECTION COMMENTED OUT ---
    // // compute shortest angular error (deg) and desired angular velocity (deg/s)
    // double errorDeg = Math.IEEEremainder(targetAngle - currAngle, 360.0);
    // double desiredVelDegPerSec = errorDeg * TURN_VEL_SCALE;
    // // clamp
    // desiredVelDegPerSec = Math.max(-TURN_MAX_VEL_DEG_PER_SEC,
    // Math.min(TURN_MAX_VEL_DEG_PER_SEC, desiredVelDegPerSec));

    // // convert to rad/s for feedforward
    // double desiredVelRadPerSec = Math.toRadians(desiredVelDegPerSec);

    // // feedforward returns volts; normalize to motor [-1,1] by battery voltage
    // double ffVolts = turnFeedforward.calculate(desiredVelRadPerSec);
    // double ff = 0.0;
    // double batteryV = RobotController.getBatteryVoltage();
    // if (batteryV > 0.001) {
    //   ff = ffVolts / batteryV;
    // }

    // double rotationSpeed = pidOutput + ff;
    
    // --- PID ONLY ---
    double rotationSpeed = pidOutput;
    rotationSpeed = Math.max(-1.0, Math.min(1.0, rotationSpeed));
    driveSubsystem.driveArcade(0, rotationSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.driveArcade(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // finish when within tolerance
    return turnPID.atSetpoint();
  }
}
