// MIT License

// Copyright (c) 2024 Isopod00

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
// Author: UMN Robotics Ri3d

package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.VisionConstants.*;

public class VisionSubsystem extends SubsystemBase {
    PhotonCamera intakeCamera, shooterCamera; // Declare the name of the camera used in the pipeline
    boolean intakeHasTarget = false, shooterHasTarget = false; // Stores whether or not a target is detected
    PhotonPipelineResult intakeResult = null, shooterResult = null; // Stores all the data that Photonvision returns
    
    public VisionSubsystem() {
        intakeCamera = new PhotonCamera(INTAKE_USB_CAMERA_NAME);
        shooterCamera = new PhotonCamera(SHOOTER_USB_CAMERA_NAME);
    }

    @Override
    public void periodic() {
        this.intakeResult = intakeCamera.getLatestResult(); // Query the latest result from PhotonVision
        this.shooterResult = shooterCamera.getLatestResult(); // Query the latest result from PhotonVision
        intakeHasTarget = (intakeResult != null) && intakeResult.hasTargets(); // update boolean
        shooterHasTarget = (shooterResult != null) && shooterResult.hasTargets(); // update boolean

        if (intakeHasTarget && intakeResult.getBestTarget() != null) {
            PhotonTrackedTarget best = intakeResult.getBestTarget();
            SmartDashboard.putNumber("Intake Best Target ID", best.getFiducialId());
            SmartDashboard.putNumber("intake t_area", best.getArea());
            SmartDashboard.putNumber("intake t_pitch", best.getPitch());
            SmartDashboard.putNumber("intake t_yaw", best.getYaw());
            SmartDashboard.putNumber("intake t_skew", best.getSkew());
            // If you want distance/InRange telemetry, compute and publish here
            SmartDashboard.putNumber("intake t_distance_est", getDistanceToIntakeTarget(best));
            SmartDashboard.putBoolean("Intake InRange", InRange(0, 5, 0, 5));
        } else {
            SmartDashboard.putNumber("Intake Best Target ID", -1);
            SmartDashboard.putNumber("intake t_area", 0);
            SmartDashboard.putNumber("intake t_pitch", 0);
            SmartDashboard.putNumber("intake t_yaw", 0);
            SmartDashboard.putNumber("intake t_skew", 0);
            SmartDashboard.putNumber("intake t_distance_est", 0);
            SmartDashboard.putBoolean("Intake InRange", false);
        }

        if (shooterHasTarget && shooterResult.getBestTarget() != null) {
            PhotonTrackedTarget best = shooterResult.getBestTarget();
            SmartDashboard.putNumber("Shooter Best Target ID", best.getFiducialId());
            SmartDashboard.putNumber("shooter t_area", best.getArea());
            SmartDashboard.putNumber("shooter t_pitch", best.getPitch());
            SmartDashboard.putNumber("shooter t_yaw", best.getYaw());
            SmartDashboard.putNumber("shooter t_skew", best.getSkew());
            // If you want distance/InRange telemetry, compute and publish here
            SmartDashboard.putNumber("shooter t_distance_est", getDistanceToShooterTarget(best));
            SmartDashboard.putBoolean("Shooter InRange", InRange(0, 5, 0, 5));
        } else {
            SmartDashboard.putNumber("Shooter Best Target ID", -1);
            SmartDashboard.putNumber("shooter t_area", 0);
            SmartDashboard.putNumber("shooter t_pitch", 0);
            SmartDashboard.putNumber("shooter t_yaw", 0);
            SmartDashboard.putNumber("shooter t_skew", 0);
            SmartDashboard.putNumber("shooter t_distance_est", 0);
            SmartDashboard.putBoolean("Shooter InRange", false);
        }

        InRange(0, 5, 0, 5); // Put to SmartDashboard whether or not the target is in range
    }


    public PhotonPipelineResult getLatestIntakeResult() {
        return this.intakeResult;
    }

    public PhotonPipelineResult getLatestShooterResult() {
        return this.shooterResult;
    }

    public PhotonTrackedTarget getIntakeTargetWithID(int id) { // Returns the apriltag target with the specified ID (if it exists)
        if (intakeResult == null) {
            return null;
        }

        List<PhotonTrackedTarget> targets = intakeResult.getTargets(); // Create a list of all currently tracked targets
        for (PhotonTrackedTarget i : targets) {
            if (i.getFiducialId() == id) { // Check the ID of each target in the list
                return i; // Found the target with the specified ID!
            }
        }
        return null; // Failed to find the target with the specified ID
    }

    public PhotonTrackedTarget getShooterTargetWithID(int id) { // Returns the apriltag target with the specified ID (if it exists)
        if (shooterResult == null) {
            return null;
        }

        List<PhotonTrackedTarget> targets = shooterResult.getTargets(); // Create a list of all currently tracked targets
        for (PhotonTrackedTarget i : targets) {
            if (i.getFiducialId() == id) { // Check the ID of each target in the list
                return i; // Found the target with the specified ID!
            }
        }
        return null; // Failed to find the target with the specified ID
    }
    
    public PhotonTrackedTarget getBestIntakeTarget() {
        if (intakeHasTarget && intakeResult != null) {
        return intakeResult.getBestTarget(); // Returns the best (closest) target
        }
        else {
            return null; // Otherwise, returns null if no targets are currently found
        }
    }

    public PhotonTrackedTarget getBestShooterTarget() {
        if (shooterHasTarget && shooterResult != null) {
        return shooterResult.getBestTarget(); // Returns the best (closest) target
        }
        else {
            return null; // Otherwise, returns null if no targets are currently found
        }
    }

    public int getTargetID(PhotonTrackedTarget target) {
        return target.getFiducialId();
    }
    public boolean getIntakeHasTarget() {
        return intakeHasTarget; // Returns whether or not a target was found
    }

    public boolean getShooterHasTarget() {
        return shooterHasTarget; // Returns whether or not a target was found
    }

    public double getDistanceToIntakeTarget(PhotonTrackedTarget target) {
        if (!intakeHasTarget || target == null) {
            return Double.NaN;
        }

        double distance = PhotonUtils.calculateDistanceToTargetMeters(
                INTAKE_CAMERA_HEIGHT_METERS,
                HUB_TARGET_HEIGHT_METERS,
                INTAKE_CAMERA_PITCH_RADIANS,
                Math.toRadians(target.getPitch())
            );
        SmartDashboard.putNumber("t_area", target.getArea());
        SmartDashboard.putNumber("t_pitch", target.getPitch());
        return distance;
    }

    public double getDistanceToShooterTarget(PhotonTrackedTarget target) {
        if (!shooterHasTarget || target == null) {
            return Double.NaN;
        }

        double distance = PhotonUtils.calculateDistanceToTargetMeters(
                SHOOTER_CAMERA_HEIGHT_METERS,
                HUB_TARGET_HEIGHT_METERS,
                SHOOTER_CAMERA_PITCH_RADIANS,
                Math.toRadians(target.getPitch())
            );
        SmartDashboard.putNumber("t_area", target.getArea());
        SmartDashboard.putNumber("t_pitch", target.getPitch());
        return distance;
    }

    public boolean InRange(double distanceThreshold, double distanceThresholdRange,
    double angleThreshold, double angleThresholdRange) {
        if (!intakeHasTarget) {
            return false;
        }
    
        PhotonTrackedTarget bestTarget = getBestIntakeTarget();
        if (bestTarget == null) {
            return false;
        }

        double distanceToTarget  = getDistanceToIntakeTarget(bestTarget);
        double angleToTarget = bestTarget.getYaw(); // Assuming yaw gives the angle

        // Check that the target is within the +/- range around the thresholds
        boolean distanceOk = Math.abs(distanceToTarget - distanceThreshold) <= distanceThresholdRange;
        boolean angleOk = Math.abs(angleToTarget - angleThreshold) <= angleThresholdRange;

        return distanceOk && angleOk;
    }
}

// I need to modify 'periodic()' to call a new function 'InRange()' that returns a boolean value if the target is within a distance and angle range 
