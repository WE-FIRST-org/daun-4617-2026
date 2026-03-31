package frc.robot.commands;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import static frc.robot.Constants.VisionConstants.*;
import static frc.robot.Constants.AimAndRangeConstants.*;

public class AimAndRangeCommand extends Command {
    private final CANDriveSubsystem m_drive;
    private final VisionSubsystem m_vision;
    
    private final PIDController turnPID = new PIDController(TURN_P, TURN_I,TURN_D);
    private final PIDController drivePID = new PIDController(DRIVE_P, DRIVE_I, DRIVE_D);

    private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(DRIVE_kS, DRIVE_kV, DRIVE_kA);
    private final SimpleMotorFeedforward turnFeedforward = new SimpleMotorFeedforward(TURN_kS, TURN_kV, TURN_kA);

    public AimAndRangeCommand(CANDriveSubsystem drive, VisionSubsystem vision) {
        m_drive = drive;
        m_vision = vision;
        // Only require drive; vision does not need to be interrupting
        addRequirements(m_drive); // Interrupts other drive commands
    }

    @Override
    public void initialize() {
        turnPID.reset();
        drivePID.reset();
    }

    @Override
    public void execute() {
        PhotonPipelineResult result = m_vision.getLatestIntakeResult();

        if (result == null || !result.hasTargets()) {
            m_drive.stop();
            SmartDashboard.putBoolean("AimAndRange/HasTarget", false);
            return;
        }
        
        PhotonTrackedTarget target = result.getBestTarget();
        if (target == null) {
            m_drive.stop();
            SmartDashboard.putBoolean("AimAndRange/HasTarget", false);
            return;
        }

        SmartDashboard.putBoolean("AimAndRange/HasTarget", true);

        double distance = m_vision.getDistanceToIntakeTarget(target); // meters (may be NaN)
        double angleDeg = target.getYaw(); // degrees

        // Defensive: ensure numeric distance
        if (Double.isNaN(distance)) {
            m_drive.stop();
            SmartDashboard.putString("AimAndRange/Distance", "NaN");
            return;
        }

        // PID measurement/setpoint
        double pidTurn = turnPID.calculate(angleDeg, 0.0); // want yaw == 0
        double pidDrive = drivePID.calculate(distance, DISTANCE_GOAL_METERS);

        // --- Turn feedforward ---
        // Map angular error (deg) to desired angular velocity (deg/s)
        double errorDeg = Math.abs(angleDeg); // angleDeg is yaw to target; we want magnitude toward 0
        double desiredAngVelDegPerSec = Math.signum(angleDeg) * Math.max(-TURN_MAX_VEL_DEG_PER_SEC,
            Math.min(TURN_MAX_VEL_DEG_PER_SEC, errorDeg * TURN_VEL_SCALE));
        double desiredAngVelRadPerSec = Math.toRadians(desiredAngVelDegPerSec);
        double ffTurnVolts = turnFeedforward.calculate(desiredAngVelRadPerSec);
        double ffTurn = ffTurnVolts / Math.max(0.001, RobotController.getBatteryVoltage());

        // --- Drive feedforward ---
        double errorDist = distance - DISTANCE_GOAL_METERS;
        double desiredLinVel = Math.max(-DRIVE_MAX_VEL_M_PER_S,
            Math.min(DRIVE_MAX_VEL_M_PER_S, errorDist * DRIVE_VEL_SCALE));
        double ffDriveVolts = driveFeedforward.calculate(desiredLinVel);
        double ffDrive = ffDriveVolts / Math.max(0.001, RobotController.getBatteryVoltage());

        double rotationSpeed = pidTurn + ffTurn;
        double forwardSpeed = pidDrive + ffDrive;

        // Clamp outputs to safe [-1,1] (or your motor input range)
        rotationSpeed = Math.max(-1.0, Math.min(1.0, rotationSpeed));
        forwardSpeed  = Math.max(-1.0, Math.min(1.0, forwardSpeed));

        SmartDashboard.putNumber("AimAndRange/DistanceMeters", distance);
        SmartDashboard.putNumber("AimAndRange/AngleDeg", angleDeg);
        SmartDashboard.putNumber("AimAndRange/ForwardSpeed", forwardSpeed);
        SmartDashboard.putNumber("AimAndRange/RotationSpeed", rotationSpeed);

        m_drive.driveArcade(forwardSpeed, rotationSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
