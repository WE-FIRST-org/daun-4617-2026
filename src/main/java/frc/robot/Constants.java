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
    // Robot geometry — measure center-to-center of wheels on your actual robot
    public static final double TRACKWIDTH_METERS = 0.5969; // measure left-to-right (~23.5 in)
    public static final double WHEELBASE_METERS  = 0.5969; // measure front-to-back (~23.5 in)

    // Pigeon V1 CAN ID
    public static final int PIGEON_ID = 0; // FIXME: set actual CAN ID

    // Maximum robot speeds — tune after mechanical validation
    public static final double MAX_VELOCITY_MPS = 4.5;            // meters/sec (L2 Mk4i + Neo Vortex)
    public static final double MAX_ANGULAR_VELOCITY_RPS = Math.PI * 2; // radians/sec

    // ── Front Left Module ──────────────────────────────────────────────────
    public static final int FL_DRIVE_ID    = 10;
    public static final int FL_STEER_ID    = 11;
    public static final int FL_ENCODER_ID  = 2;
    public static final double FL_STEER_OFFSET = 0.13623046875; 

    // ── Front Right Module ─────────────────────────────────────────────────
    public static final int FR_DRIVE_ID    = 3;
    public static final int FR_STEER_ID    = 7;
    public static final int FR_ENCODER_ID  = 9;
    public static final double FR_STEER_OFFSET = -0.181640625;

    // ── Back Left Module ───────────────────────────────────────────────────
    public static final int BL_DRIVE_ID    = 62;
    public static final int BL_STEER_ID    = 61;
    public static final int BL_ENCODER_ID  = 8;
    public static final double BL_STEER_OFFSET = 0.04296875; 

    // ── Back Right Module ──────────────────────────────────────────────────
    public static final int BR_DRIVE_ID    = 4;
    public static final int BR_STEER_ID    = 6;
    public static final int BR_ENCODER_ID  = 12;
    public static final double BR_STEER_OFFSET = -0.34716796875;

    // ── Mk4i Physical Specs ────────────────────────────────────────────────
    // Drive gear ratio: L1=8.14, L2=6.75 (default), L3=6.12
    public static final double DRIVE_GEAR_RATIO  = 6.75;           // FIXME: match your Mk4i L-ratio
    public static final double STEER_GEAR_RATIO  = 150.0 / 7.0;   // 21.43:1, fixed for all Mk4i
    public static final double WHEEL_DIAMETER_METERS = 0.1016;     // 4 inches

    // SparkFlex encoder conversion factors
    public static final double DRIVE_ENCODER_POSITION_FACTOR = (WHEEL_DIAMETER_METERS * Math.PI) / DRIVE_GEAR_RATIO;  // meters per motor rotation
    public static final double DRIVE_ENCODER_VELOCITY_FACTOR = DRIVE_ENCODER_POSITION_FACTOR / 60.0;// m/s per RPM

    public static final double STEER_ENCODER_POSITION_FACTOR =
        (2 * Math.PI) / STEER_GEAR_RATIO;   // radians per motor rotation
    public static final double STEER_ENCODER_VELOCITY_FACTOR =
        STEER_ENCODER_POSITION_FACTOR / 60.0; // rad/s per RPM

    // Steer PID gains (tune on robot)
    public static final double STEER_P = 1.0;
    public static final double STEER_I = 0.0;
    public static final double STEER_D = 0.0;

    // Current limits
    public static final int DRIVE_MOTOR_CURRENT_LIMIT = 60;
    public static final int STEER_MOTOR_CURRENT_LIMIT = 20;
  }

  public static final class FuelConstants {
    // Motor controller IDs for Fuel Mechanism motors
    public static final int FEEDER_MOTOR_ID = 14;
    public static final int INTAKE_LAUNCHER_MOTOR_ID = 15;

    // Current limit and nominal voltage for fuel mechanism motors.
    public static final int FEEDER_MOTOR_CURRENT_LIMIT = 60;
    public static final int LAUNCHER_MOTOR_CURRENT_LIMIT = 60;

    // Voltage values for various fuel operations. These values may need to be tuned
    // based on exact robot construction.
    // See the Software Guide for tuning information
    public static final double INTAKING_FEEDER_VOLTAGE = -12;
    public static final double INTAKING_INTAKE_VOLTAGE = 10;
    public static final double LAUNCHING_FEEDER_VOLTAGE = 9;
    public static final double LAUNCHING_LAUNCHER_VOLTAGE = 10.6;
    public static final double SPIN_UP_FEEDER_VOLTAGE = -6;
    public static final double SPIN_UP_SECONDS = 1;
  }

  public static final class VisionConstants {
    // VISION CONSTANTS
    public static final String USB_CAMERA_NAME = "USB_Camera-10.48.14.5"; //VisionSubsystem.java MIT License text there
  }

  public static final class OperatorConstants {
    // Port constants for driver and operator controllers. These should match the
    // values in the Joystick tab of the Driver Station software
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;

    // This value is multiplied by the joystick value when rotating the robot to
    // help avoid turning too fast and beign difficult to control
    public static final double DRIVE_SCALING = .7;
    public static final double ROTATION_SCALING = .8;
  }
}
