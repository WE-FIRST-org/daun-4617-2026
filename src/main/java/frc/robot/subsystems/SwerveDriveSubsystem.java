package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.DriveConstants.*;

/**
 * Swerve drivetrain subsystem for an Mk4i chassis with:
 *   • 4× Neo Vortex drive motors (SparkFlex)
 *   • 4× Neo Vortex steer motors (SparkFlex)
 *   • 4× CTRE CANCoder absolute encoders
 *   • Pigeon V1 gyroscope (on-board)
 *
 * Adapted from the SDS swerve-template; rewritten for WPILib 2026.
 */
public class SwerveDriveSubsystem extends SubsystemBase {

    // ── Kinematics ────────────────────────────────────────────────────────
    // Module positions relative to robot center (FL, FR, BL, BR)
    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
        new Translation2d( WHEELBASE_METERS / 2.0,  TRACKWIDTH_METERS / 2.0),  // Front Left
        new Translation2d( WHEELBASE_METERS / 2.0, -TRACKWIDTH_METERS / 2.0),  // Front Right
        new Translation2d(-WHEELBASE_METERS / 2.0,  TRACKWIDTH_METERS / 2.0),  // Back Left
        new Translation2d(-WHEELBASE_METERS / 2.0, -TRACKWIDTH_METERS / 2.0)   // Back Right
    );

    // ── Gyroscope ─────────────────────────────────────────────────────────
    private final PigeonIMU m_pigeon = new PigeonIMU(PIGEON_ID);

    // ── Modules ───────────────────────────────────────────────────────────
    // driveInverted: FL and BL are typically inverted on SDS modules — verify on your robot
    private final SwerveModule m_frontLeft  = new SwerveModule(
        FL_DRIVE_ID, FL_STEER_ID, FL_ENCODER_ID, FL_STEER_OFFSET, false);
    private final SwerveModule m_frontRight = new SwerveModule(
        FR_DRIVE_ID, FR_STEER_ID, FR_ENCODER_ID, FR_STEER_OFFSET, true);
    private final SwerveModule m_backLeft   = new SwerveModule(
        BL_DRIVE_ID, BL_STEER_ID, BL_ENCODER_ID, BL_STEER_OFFSET, false);
    private final SwerveModule m_backRight  = new SwerveModule(
        BR_DRIVE_ID, BR_STEER_ID, BR_ENCODER_ID, BR_STEER_OFFSET, true);

    // ── Odometry ──────────────────────────────────────────────────────────
    private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
        m_kinematics,
        getGyroscopeRotation(),
        getModulePositions()
    );

    private final Field2d m_field = new Field2d();

    public SwerveDriveSubsystem() {
        SmartDashboard.putData("Field", m_field);
        zeroGyroscope();
    }

    // ── Public interface ──────────────────────────────────────────────────

    /**
     * Drives the robot at the given chassis speeds.
     * Call with field-relative speeds from {@link ChassisSpeeds#fromFieldRelativeSpeeds}.
     */
    public void drive(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_MPS);

        m_frontLeft.setDesiredState(states[0], MAX_VELOCITY_MPS);
        m_frontRight.setDesiredState(states[1], MAX_VELOCITY_MPS);
        m_backLeft.setDesiredState(states[2], MAX_VELOCITY_MPS);
        m_backRight.setDesiredState(states[3], MAX_VELOCITY_MPS);
    }

    /** Stops all modules. */
    public void stop() {
        m_frontLeft.stop();
        m_frontRight.stop();
        m_backLeft.stop();
        m_backRight.stop();
    }

    /** Resets the Pigeon yaw to zero so the current heading is treated as "forward." */
    public void zeroGyroscope() {
        m_pigeon.setYaw(0.0);
    }

    /** Returns the robot's current heading as a {@link Rotation2d}. */
    public Rotation2d getGyroscopeRotation() {
        return Rotation2d.fromDegrees(m_pigeon.getYaw());
    }

    /** Returns the robot's estimated pose on the field. */
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    /** Resets the odometry to the given pose. */
    public void resetOdometry(Pose2d pose) {
        m_odometry.resetPosition(getGyroscopeRotation(), getModulePositions(), pose);
    }

    // ── Periodic ──────────────────────────────────────────────────────────

    @Override
    public void periodic() {
        m_odometry.update(getGyroscopeRotation(), getModulePositions());
        m_field.setRobotPose(m_odometry.getPoseMeters());

        SmartDashboard.putNumber("Gyro Yaw (deg)", m_pigeon.getYaw());
        SmartDashboard.putString("Robot Pose", m_odometry.getPoseMeters().toString());
    }

    // ── Helpers ───────────────────────────────────────────────────────────

    private SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
        };
    }
}
