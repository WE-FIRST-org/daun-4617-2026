package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import static frc.robot.Constants.DriveConstants.*;

/**
 * Represents one Mk4i swerve module:
 *   Drive motor  — Neo Vortex via SparkFlex (open-loop voltage)
 *   Steer motor  — Neo Vortex via SparkFlex (closed-loop position PID)
 *   Steer encoder — CTRE CANCoder (absolute, used only for initial sync)
 */
public class SwerveModule {

    private final SparkFlex m_driveMotor;
    private final SparkFlex m_steerMotor;
    private final CANcoder  m_steerEncoder;

    private final RelativeEncoder          m_driveEncoder;
    private final RelativeEncoder          m_steerRelEncoder;
    private final SparkClosedLoopController m_steerPID;

    private final double m_steerOffsetRad;

    /**
     * @param driveMotorId   CAN ID of the SparkFlex driving the wheel
     * @param steerMotorId   CAN ID of the SparkFlex turning the module
     * @param steerEncoderId CAN ID of the CTRE CANCoder
     * @param steerOffsetRad Absolute angle offset (radians) so forward = 0
     * @param driveInverted  True if drive motor positive = wheel backward
     */
    public SwerveModule(int driveMotorId, int steerMotorId,
                        int steerEncoderId, double steerOffsetRad,
                        boolean driveInverted) {
        m_steerOffsetRad = steerOffsetRad;

        // ── Drive motor ───────────────────────────────────────────────────
        m_driveMotor = new SparkFlex(driveMotorId, MotorType.kBrushless);
        SparkFlexConfig driveConfig = new SparkFlexConfig();
        driveConfig.inverted(driveInverted);
        driveConfig.smartCurrentLimit(DRIVE_MOTOR_CURRENT_LIMIT);
        driveConfig.voltageCompensation(12.0);
        driveConfig.encoder
            .positionConversionFactor(DRIVE_ENCODER_POSITION_FACTOR)
            .velocityConversionFactor(DRIVE_ENCODER_VELOCITY_FACTOR);
        m_driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_driveEncoder = m_driveMotor.getEncoder();
        m_driveEncoder.setPosition(0);

        // ── Steer motor ───────────────────────────────────────────────────
        m_steerMotor = new SparkFlex(steerMotorId, MotorType.kBrushless);
        SparkFlexConfig steerConfig = new SparkFlexConfig();
        steerConfig.inverted(true); // Mk4i steer is inverted
        steerConfig.smartCurrentLimit(STEER_MOTOR_CURRENT_LIMIT);
        steerConfig.voltageCompensation(12.0);
        steerConfig.encoder
            .positionConversionFactor(STEER_ENCODER_POSITION_FACTOR)
            .velocityConversionFactor(STEER_ENCODER_VELOCITY_FACTOR);
        steerConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(STEER_P, STEER_I, STEER_D)
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(-Math.PI, Math.PI);
        m_steerMotor.configure(steerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_steerRelEncoder = m_steerMotor.getEncoder();
        m_steerPID = m_steerMotor.getClosedLoopController();

        // ── CANcoder (Phoenix 6) ──────────────────────────────────────────
        m_steerEncoder = new CANcoder(steerEncoderId);

        // Sync the SparkFlex relative encoder to the absolute CANCoder position on startup.
        // This means the PID always works in radians from the true wheel angle.
        m_steerRelEncoder.setPosition(getAbsoluteAngleRad());
    }

    /** Returns the absolute wheel angle in radians, corrected by the steer offset. */
    private double getAbsoluteAngleRad() {
        // Phoenix 6 CANcoder returns rotations (-0.5 to 0.5) as a unit type; use getValueAsDouble()
        double rotations = m_steerEncoder.getAbsolutePosition().getValueAsDouble();
        double angle = rotations * 2.0 * Math.PI - m_steerOffsetRad;
        return MathUtil.angleModulus(angle);
    }

    /** Current module state (wheel speed m/s and steer angle). */
    public SwerveModuleState getState() {
        return new SwerveModuleState(
            m_driveEncoder.getVelocity(),
            new Rotation2d(m_steerRelEncoder.getPosition())
        );
    }

    /** Current module position (wheel distance m and steer angle) — used for odometry. */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            m_driveEncoder.getPosition(),
            new Rotation2d(m_steerRelEncoder.getPosition())
        );
    }

    /**
     * Commands this module to the desired state.
     * Optimizes so the wheel never turns more than 90°.
     *
     * @param desiredState target speed and angle
     * @param maxVelocityMps robot's configured maximum velocity, used to scale to voltage
     */
    public void setDesiredState(SwerveModuleState desiredState, double maxVelocityMps) {
        // Optimize: flip drive direction instead of spinning steer >90°
        SwerveModuleState optimized = SwerveModuleState.optimize(
            desiredState, new Rotation2d(m_steerRelEncoder.getPosition()));

        // Drive: open-loop voltage proportional to fraction of max speed
        double driveVoltage = (optimized.speedMetersPerSecond / maxVelocityMps) * 12.0;
        m_driveMotor.setVoltage(driveVoltage);

        // Steer: closed-loop position PID (radians, wrapping enabled)
        m_steerPID.setReference(
            optimized.angle.getRadians(), ControlType.kPosition);
    }

    /** Stops both motors. */
    public void stop() {
        m_driveMotor.setVoltage(0);
        m_steerMotor.setVoltage(0);
    }
}
