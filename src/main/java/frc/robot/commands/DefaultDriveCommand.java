package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDriveSubsystem;

import java.util.function.DoubleSupplier;

/**
 * Default field-oriented swerve drive command.
 *
 * Suppliers provide velocity in m/s (translation) and rad/s (rotation),
 * already scaled and deadbanded by the caller (RobotContainer).
 *
 * Adapted from the SDS swerve-template; updated for WPILib 2026.
 */
public class DefaultDriveCommand extends Command {

    private final SwerveDriveSubsystem m_driveSubsystem;

    private final DoubleSupplier m_translationXSupplier; // m/s, field-relative forward
    private final DoubleSupplier m_translationYSupplier; // m/s, field-relative left
    private final DoubleSupplier m_rotationSupplier;     // rad/s, CCW positive

    public DefaultDriveCommand(SwerveDriveSubsystem driveSubsystem,
                               DoubleSupplier translationXSupplier,
                               DoubleSupplier translationYSupplier,
                               DoubleSupplier rotationSupplier) {
        m_driveSubsystem = driveSubsystem;
        m_translationXSupplier = translationXSupplier;
        m_translationYSupplier = translationYSupplier;
        m_rotationSupplier     = rotationSupplier;

        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        m_driveSubsystem.drive(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                m_translationXSupplier.getAsDouble(),
                m_translationYSupplier.getAsDouble(),
                m_rotationSupplier.getAsDouble(),
                m_driveSubsystem.getGyroscopeRotation()
            )
        );
    }

    @Override
    public void end(boolean interrupted) {
        m_driveSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}
