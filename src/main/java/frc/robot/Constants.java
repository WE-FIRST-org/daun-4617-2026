// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Motor controller IDs for drivetrain motors
    public static final int LEFT_LEADER_ID = 1;
    public static final int LEFT_FOLLOWER_ID = 2;
    public static final int RIGHT_LEADER_ID = 3;
    public static final int RIGHT_FOLLOWER_ID = 4;

    // Current limit for drivetrain motors. 60A is a reasonable maximum to reduce
    // likelihood of tripping breakers or damaging CIM motors
    public static final int DRIVE_MOTOR_CURRENT_LIMIT = 60;

    public static final double GEAR_RATIO = 10.71;
    public static final double WHEEL_DIAMETER_METERS = 0.1524;
  }

  public static final class FuelConstants {
    // Motor controller IDs for Fuel Mechanism motors
    public static final int FEEDER_MOTOR_ID = 6;
    public static final int INTAKE_LAUNCHER_MOTOR_ID = 5;

    // Current limit and nominal voltage for fuel mechanism motors.
    public static final int FEEDER_MOTOR_CURRENT_LIMIT = 60;
    public static final int LAUNCHER_MOTOR_CURRENT_LIMIT = 60;

    // Voltage values for various fuel operations. These values may need to be tuned
    // based on exact robot construction.
    // See the Software Guide for tuning information
    public static final double INTAKING_FEEDER_VOLTAGE = 12;
    public static final double INTAKING_INTAKE_VOLTAGE = -8;
    public static final double LAUNCHING_FEEDER_VOLTAGE = 9;
    public static final double LAUNCHING_LAUNCHER_VOLTAGE = -8.17; // kitbot template: -10.21
    public static final double SPIN_UP_FEEDER_VOLTAGE = -6;
    public static final double SPIN_UP_SECONDS = 3; 
  }

  public static final class OperatorConstants {
    // Port constants for driver and operator controllers. These should match the
    // values in the Joystick tab of the Driver Station software
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;

    // This value is multiplied by the joystick value when rotating the robot to
    // help avoid turning too fast and beign difficult to control
    public static final double DRIVE_SCALING = .64;
    public static final double ROTATION_SCALING = .52;
    // Maximum boost multiplier applied to forward speed when the trigger is fully pressed.
    // Effective forward scaling = DRIVE_SCALING * (1 + trigger*(DRIVE_BOOST_FACTOR-1))
    public static final double DRIVE_BOOST_FACTOR = 1.5;
  }

  public static final class VisionConstants {
    // VISION CONSTANTS
    //make sure to change it back into 46.17 after logic implementation, access raspi's static ip address via "sudo nmcui" (Network Managemer Command line User Interface)
    //router ip address 10.TE.AM.1 (eg 10.46.17.1)
    // public static final String USB_CAMERA_NAME = "USB_Camera-10.46.17.42"; //VisionSubsystem.java MIT License text there
    public static final String INTAKE_USB_CAMERA_NAME = "Microsoft_LifeCam_HD-3000"; //VisionSubsystem.java MIT License text there
    public static final String SHOOTER_USB_CAMERA_NAME = "Microsoft_LifeCam_HD-3000 (1)"; //VisionSubsystem.java MIT License text there

    // April tag heights of game pieces
    public static final double HUB_TARGET_HEIGHT_METERS = 1.124;
    public static final double TOWER_TARGET_HEIGHT_METERS = 0.5525;
    public static final double OUTPOST_TARGET_HEIGHT_METERS = 0.5525;
    public static final double TRENCH_TARGET_HEIGHT_METERS = 0.889;

    // Camera height and pitch
    public static final double INTAKE_CAMERA_HEIGHT_METERS = 0.434;
    public static final double INTAKE_CAMERA_PITCH_DEGREES = 30;
    public static final double INTAKE_CAMERA_PITCH_RADIANS = Math.toRadians(INTAKE_CAMERA_PITCH_DEGREES);

    public static final double SHOOTER_CAMERA_HEIGHT_METERS = 0.25;
    public static final double SHOOTER_CAMERA_PITCH_DEGREES = 38;
    public static final double SHOOTER_CAMERA_PITCH_RADIANS = Math.toRadians(INTAKE_CAMERA_PITCH_DEGREES);

    public static final double DISTANCE_GOAL_METERS = 0.1;
  }

  public static final class AimAndRangeConstants {
    // PID values for turning and driving forward
    public static final double TURN_P = 0.02; // was 0.01
    public static final double TURN_I = 0;
    public static final double TURN_D = 0;

    public static final double DRIVE_P = 0.52; // was 0.1
    public static final double DRIVE_I = 0;
    public static final double DRIVE_D = 0;

    public static final double TURN_kS = 0.2;
    public static final double TURN_kV = 2;
    public static final double TURN_kA = 0.2;
  // Velocity profiling for rotational feedforward (deg error -> deg/s desired)
  // desiredVelDegPerSec = clamp(errorDeg * TURN_VEL_SCALE, -TURN_MAX_VEL_DEG_PER_SEC, TURN_MAX_VEL_DEG_PER_SEC)
  public static final double TURN_VEL_SCALE = 2.0; // deg/s per degree of error
  public static final double TURN_MAX_VEL_DEG_PER_SEC = 180.0; // clamp max angular speed

    public static final double DRIVE_kS = 0.2;
    public static final double DRIVE_kV = 2;
    public static final double DRIVE_kA = 0.1;
    // Velocity profiling for linear feedforward (m error -> m/s desired)
    // desiredVelMPerS = clamp(errorM * DRIVE_VEL_SCALE, -DRIVE_MAX_VEL_M_PER_S, DRIVE_MAX_VEL_M_PER_S)
    public static final double DRIVE_VEL_SCALE = 1.0; // m/s per meter of error
    public static final double DRIVE_MAX_VEL_M_PER_S = 3.0; // clamp max linear speed
  }
}
