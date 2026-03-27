// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.InvertDrive;
import static frc.robot.Constants.DriveConstants.*;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;

public class CANDriveSubsystem extends SubsystemBase {
  private SparkMax leftLeader;
  private SparkMax leftFollower;
  private SparkMax rightLeader;
  private SparkMax rightFollower;

  private DifferentialDrive drive;
  // telemetry: last commanded values
  private double lastXSpeed = 0.0;
  private double lastZRotation = 0.0;

  private final MutVoltage m_appliedVoltage = Volts.mutable(0);
  private final MutDistance m_distance = Meters.mutable(0);
  private final MutLinearVelocity m_linearVelocity = MetersPerSecond.mutable(0);
  private final MutAngularVelocity m_angularVelocity = DegreesPerSecond.mutable(0);
  private final MutAngle m_angle = Degrees.mutable(0);

  private IMUSubsystem imu;

  public CANDriveSubsystem(IMUSubsystem imu) {
    this.imu = imu;
    // create brushless motors for drive
    leftLeader = new SparkMax(LEFT_LEADER_ID, MotorType.kBrushless);
    leftFollower = new SparkMax(LEFT_FOLLOWER_ID, MotorType.kBrushless);
    rightLeader = new SparkMax(RIGHT_LEADER_ID, MotorType.kBrushless);
    rightFollower = new SparkMax(RIGHT_FOLLOWER_ID, MotorType.kBrushless);

    // set up differential drive class
    drive = new DifferentialDrive(leftLeader, rightLeader);

    // Set can timeout. Because this project only sets parameters once on
    // construction, the timeout can be long without blocking robot operation. Code
    // which sets or gets parameters during operation may need a shorter timeout.
    leftLeader.setCANTimeout(250);
    rightLeader.setCANTimeout(250);
    leftFollower.setCANTimeout(250);
    rightFollower.setCANTimeout(250);

    // Create the configuration to apply to motors. Voltage compensation
    // helps the robot perform more similarly on different
    // battery voltages (at the cost of a little bit of top speed on a fully charged
    // battery). The current limit helps prevent tripping
    // breakers.
    SparkMaxConfig config = new SparkMaxConfig();
    config.voltageCompensation(12);
    config.smartCurrentLimit(DRIVE_MOTOR_CURRENT_LIMIT);

    config.encoder.positionConversionFactor((Math.PI*WHEEL_DIAMETER_METERS)/GEAR_RATIO);
    config.encoder.velocityConversionFactor((Math.PI*WHEEL_DIAMETER_METERS)/(60*GEAR_RATIO));

    // Set configuration to follow each leader and then apply it to corresponding
    // follower. Resetting in case a new controller is swapped
    // in and persisting in case of a controller reset due to breaker trip
    config.follow(leftLeader);
    leftFollower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    config.follow(rightLeader);
    rightFollower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Remove following, then apply config to right leader
    config.disableFollowerMode();
    rightLeader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // Set config to inverted and then apply to left leader. Set Left side inverted
    // so that postive values drive both sides forward
    config.inverted(true);
    leftLeader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  private final SysIdRoutine m_linearRoutine = new SysIdRoutine(
    new SysIdRoutine.Config(),
    new SysIdRoutine.Mechanism(
        (voltage) -> {
          leftLeader.setVoltage(voltage.in(Volts)); // Applied voltage
          rightLeader.setVoltage(voltage.in(Volts)); // Applied voltage
          SmartDashboard.putNumber("linear routine", voltage.in(Volts));
          drive.feed();
        },
        (log) -> {
          log.motor("drive-left")
            .voltage(
              m_appliedVoltage.mut_replace(
                leftLeader.get() * RobotController.getBatteryVoltage(), Volts))
            .linearPosition(m_distance.mut_replace(leftLeader.getEncoder().getPosition(), Meters))
            .linearVelocity(
              m_linearVelocity.mut_replace(leftLeader.getEncoder().getVelocity(), MetersPerSecond));
          
          log.motor("drive-right")
            .voltage(
              m_appliedVoltage.mut_replace(
                rightLeader.get() * RobotController.getBatteryVoltage(), Volts))
            .linearPosition(m_distance.mut_replace(rightLeader.getEncoder().getPosition(), Meters))
            .linearVelocity(
              m_linearVelocity.mut_replace(rightLeader.getEncoder().getVelocity(), MetersPerSecond));
        }, 
        this
      )
  );

  private final SysIdRoutine m_angularRoutine = new SysIdRoutine(
    new SysIdRoutine.Config(), 
    new SysIdRoutine.Mechanism(
        (voltage) -> {
          leftLeader.setVoltage(voltage.in(Volts)); // Applied voltage
          rightLeader.setVoltage(voltage.unaryMinus().in(Volts)); // Applied voltage
          SmartDashboard.putNumber("angular routine", voltage.in(Volts));
          drive.feed();
        },
        (log) -> {
          log.motor("drive-left")
            .voltage(
              m_appliedVoltage.mut_replace(
                leftLeader.get() * RobotController.getBatteryVoltage(), Volts))
            .angularPosition(m_angle.mut_replace(leftLeader.getEncoder().getPosition(), Degrees))
            .angularVelocity(
              m_angularVelocity.mut_replace(imu.getAngularVelZ(), DegreesPerSecond));
          
          log.motor("drive-right")
            .voltage(
              m_appliedVoltage.mut_replace(
                rightLeader.get() * RobotController.getBatteryVoltage(), Volts))
            .angularPosition(m_angle.mut_replace(rightLeader.getEncoder().getPosition(), Degrees))
            .angularVelocity(
              m_angularVelocity.mut_replace(imu.getAngularVelZ(), DegreesPerSecond));
        },
        this
      )
  );

  public Command sysIdQuasistaticLinear(SysIdRoutine.Direction direction) {
      return m_linearRoutine.quasistatic(direction);
  }

  public Command sysIdDynamicLinear(SysIdRoutine.Direction direction) {
      return m_linearRoutine.dynamic(direction);
  }

  public Command sysIdQuasistaticAngular(SysIdRoutine.Direction direction) {
      return m_angularRoutine.quasistatic(direction);
  }

  public Command sysIdDynamicAngular(SysIdRoutine.Direction direction) {
      return m_angularRoutine.dynamic(direction);
  }

  @Override
  public void periodic() {
    boolean isInverted = InvertDrive.getInvertedStatus();
    String front;
    if (isInverted) {
      front = "Intake";
    } else {
      front = "Shooter";
    }
    SmartDashboard.putString("Front is: ", front);

    // Publish commanded inputs
    SmartDashboard.putNumber("drive/lastXSpeed", lastXSpeed);
    SmartDashboard.putNumber("drive/lastZRotation", lastZRotation);

    // Left leader telemetry
    try {
      SmartDashboard.putNumber("drive/leftLeader/current", leftLeader.getOutputCurrent());
      SmartDashboard.putNumber("drive/leftLeader/busVoltage", leftLeader.getBusVoltage());
      SmartDashboard.putNumber("drive/leftLeader/appliedOutput", leftLeader.getAppliedOutput());
  // temperature telemetry not available on this SparkMax wrapper
      SmartDashboard.putNumber("drive/leftLeader/encoderPosition", leftLeader.getEncoder().getPosition());
      SmartDashboard.putNumber("drive/leftLeader/encoderVelocity", leftLeader.getEncoder().getVelocity());
    } catch (Exception e) {
      // If any of the calls are not available on the vendor library, don't crash the robot; just skip telemetry
      SmartDashboard.putString("drive/leftLeader/telemetryError", e.getMessage());
    }

    // Right leader telemetry
    try {
      SmartDashboard.putNumber("drive/rightLeader/current", rightLeader.getOutputCurrent());
      SmartDashboard.putNumber("drive/rightLeader/busVoltage", rightLeader.getBusVoltage());
      SmartDashboard.putNumber("drive/rightLeader/appliedOutput", rightLeader.getAppliedOutput());
  // temperature telemetry not available on this SparkMax wrapper
      SmartDashboard.putNumber("drive/rightLeader/encoderPosition", rightLeader.getEncoder().getPosition());
      SmartDashboard.putNumber("drive/rightLeader/encoderVelocity", rightLeader.getEncoder().getVelocity());
    } catch (Exception e) {
      SmartDashboard.putString("drive/rightLeader/telemetryError", e.getMessage());
    }

    // Followers telemetry
    try {
      SmartDashboard.putNumber("drive/leftFollower/current", leftFollower.getOutputCurrent());
      SmartDashboard.putNumber("drive/rightFollower/current", rightFollower.getOutputCurrent());
    } catch (Exception e) {
      SmartDashboard.putString("drive/followers/telemetryError", e.getMessage());
    }
  }

  public void driveArcade(double xSpeed, double zRotation) {
    // store last commanded values for telemetry
    lastXSpeed = xSpeed;
    lastZRotation = zRotation;
    drive.arcadeDrive(xSpeed, zRotation);
  }

  /**
   * 
   */
  public void stop() {
    drive.stopMotor();
  }

  public void resetEnconders() {
    leftLeader.getEncoder().setPosition(0);
    rightLeader.getEncoder().setPosition(0);
  }

}
