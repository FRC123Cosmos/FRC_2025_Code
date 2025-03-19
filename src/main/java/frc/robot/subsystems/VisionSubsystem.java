package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase {
    private final PhotonCamera camera = new PhotonCamera("GSC_BLACK");
    private PhotonPipelineResult latestResult = new PhotonPipelineResult();

    private final String cameraName = "GSC_BLACK";

    
    // Camera position relative to robot center (in meters)
    private final Translation2d cameraOffset;
    // x (forward), y (left/right)
    private final Rotation2d cameraYawOffset; // Camera's yaw relative to robot heading
    private final double cameraHeight; // z (height) in meters
    private final double cameraPitch; // Pitch angle in degrees

        
    private double rawYaw;
    private double target_x=0;
    public static double target_y;
    private double target_z=0;

    public VisionSubsystem() {
        CommandScheduler.getInstance().registerSubsystem(this);
        
        cameraOffset = new Translation2d(VisionConstants.cameraXOffset, VisionConstants.cameraYOffset);
        cameraYawOffset = Rotation2d.fromDegrees(VisionConstants.cameraYawOffset);
        cameraHeight = VisionConstants.cameraHeight;
        cameraPitch = VisionConstants.cameraPitch;

        target_y = 1;
    }

    @Override
    public void periodic() {
        // query camera
        latestResult = camera.getLatestResult();
        // latestResultG = cameraG.getLatestResult();
        boolean hasTarget = latestResult.hasTargets();
        // avoid null pointer exception if no tracked target
        // if ( hasTargetB ) {
            PhotonTrackedTarget target = latestResult.getBestTarget();
            int targetID = target.getFiducialId();
            double poseAmbiguity = target.getPoseAmbiguity();
            Transform3d bestCameraToTargetPose = target.getBestCameraToTarget();

            rawYaw = target.getYaw();
            target_x = bestCameraToTargetPose.getX();
            target_y = bestCameraToTargetPose.getY();
            target_z = bestCameraToTargetPose.getZ();
            // mpk - comment out after verifing target values
            // SmartDashboard.putNumber("targetID",targetID);
            // SmartDashboard.putNumber("Ambiguity",poseAmbiguity);
            // SmartDashboard.putNumber("cameraB Yaw", rawYaw);
            SmartDashboard.putNumber("target_x", target_x);
            SmartDashboard.putNumber("target_y", target_y);
            SmartDashboard.putNumber("target_z", target_z);
        // }

        // rawYaw = latestResultB.getBestTarget().getYaw();

        // SmartDashboard.putNumber("camera Yaw", rawYawB);

        SmartDashboard.putBoolean("isLeftAligned", isLeftAlignB());
    }


    public boolean hasTargetB() {
        return latestResult.hasTargets();      // mpk - should be periodic result via public gettter/setter interface?
    }

    /**
     * Get yaw adjusted for camera offset
     * @param robotHeading Current robot heading (Rotation2d)
     * @return Yaw in degrees relative to robot frame
     */
    public double getTargetYawAdjustedB(Rotation2d robotHeading) {
        if (hasTargetB()) {
            double rawYaw = latestResult.getBestTarget().getYaw();
            // Adjust yaw for camera's orientation and position
            Rotation2d adjustedYawB = Rotation2d.fromDegrees(rawYaw).plus(cameraYawOffset).minus(robotHeading);
            return adjustedYawB.getDegrees();
        }
        return 0.0;
    }

    public boolean isLeftAlignB(){
        if (hasTargetB()) {
            return (rawYaw > VisionConstants.leftAlignRangeLeftInterval) && (rawYaw < VisionConstants.leftAlignRangeRightInterval);
        }
        else{
            return false;
        }
    }

    public double getTargetPitchB() {
        if (hasTargetB()) {
            return latestResult.getBestTarget().getPitch();
        }
        return 0.0;
    }

    public double getTargetAreaB() {
        if (hasTargetB()) {
            return latestResult.getBestTarget().getArea();
        }
        return 0.0;
    }

    /**
     * Estimate distance to target, accounting for camera height and pitch
     * @param targetHeight Height of target from ground (meters)
     * @return Distance in meters, -1 if no target
     */
    public double getDistanceToTargetB(double targetHeight) {
        if (hasTargetB()) {
            double pitch = getTargetPitchB();
            double totalPitch = Math.toRadians(cameraPitch + pitch);
            return (targetHeight - cameraHeight) / Math.tan(totalPitch);
        }
        return -1.0;
    }

    public PhotonTrackedTarget getBestTargetB() {
        if (hasTargetB()) {
            return latestResult.getBestTarget();
        }
        return null;
    }

    public Translation2d getCameraOffset() {
        return cameraOffset;
    }

    public Rotation2d getCameraYawOffset() {
        return cameraYawOffset;
    }
}
