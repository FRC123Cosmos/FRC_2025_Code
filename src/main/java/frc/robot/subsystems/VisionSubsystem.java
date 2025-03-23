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

// public class VisionSubsystem extends SubsystemBase {
//     private final PhotonCamera camera = new PhotonCamera("GSC_BLACK");
//     private PhotonPipelineResult latestResult = new PhotonPipelineResult();
    
//     private final String cameraName = "GSC_BLACK";

    
//     // Camera position relative to robot center (in meters)
//     private final Translation2d cameraOffset = new Translation2d(VisionConstants.cameraXOffset, VisionConstants.cameraYOffset);
//     // x (forward), y (left/right)
//     private final Rotation2d cameraYawOffset = Rotation2d.fromDegrees(VisionConstants.cameraYawOffset); // Camera's yaw relative to robot heading
//     private final double cameraHeight = VisionConstants.cameraHeight; // z (height) in meters
//     private final double cameraPitch = VisionConstants.cameraPitch; // Pitch angle in degrees

//     private PhotonTrackedTarget target;
//     private double rawYaw = 0;
//     private double target_x = 0;
//     public static double target_y = 0;
//     private double target_z = 0; 
//     private int targetID;
//     private boolean hasTarget;
//     private double poseAmbiguity;
//     private Transform3d bestCameraToTargetPose;

//     public VisionSubsystem() {
//         CommandScheduler.getInstance().registerSubsystem(this);
//         }

//     @Override
//     public void periodic() {
//         // query camera
//         latestResult = camera.getLatestResult();
//         // latestResultG = cameraG.getLatestResult();
//         hasTarget = latestResult.hasTargets();
//         // avoid null pointer exception if no tracked target
//         // if ( hasTargetB ) {
//             target = latestResult.getBestTarget();
//             targetID = target.getFiducialId();
//             poseAmbiguity = target.getPoseAmbiguity();
//             bestCameraToTargetPose = target.getBestCameraToTarget();

//             rawYaw = target.getYaw();
//             target_x = bestCameraToTargetPose.getX();
//             target_y = bestCameraToTargetPose.getY();
//             target_z = bestCameraToTargetPose.getZ();
//             // mpk - comment out after verifing target values
//             // SmartDashboard.putNumber("targetID",targetID);
//             // SmartDashboard.putNumber("Ambiguity",poseAmbiguity);
//             // SmartDashboard.putNumber("cameraB Yaw", rawYaw);
//             // SmartDashboard.putNumber("target_x", target_x);
//             // SmartDashboard.putNumber("target_y", target_y);
//             // SmartDashboard.putNumber("target_z", target_z);
//         // }

//         // rawYaw = latestResultB.getBestTarget().getYaw();

//         // SmartDashboard.putNumber("camera Yaw", rawYawB);

//         // SmartDashboard.putBoolean("isLeftAligned", isLeftAlign());
//     }


//     public boolean hasTarget() {
//         return latestResult.hasTargets();      // mpk - should be periodic result via public gettter/setter interface?
//     }

//     /**
//      * Get yaw adjusted for camera offset
//      * @param robotHeading Current robot heading (Rotation2d)
//      * @return Yaw in degrees relative to robot frame
//      */
//     public double getTargetYawAdjusted(Rotation2d robotHeading) {
//         if (hasTarget()) {
//             // Adjust yaw for camera's orientation and position
//             Rotation2d adjustedYawB = Rotation2d.fromDegrees(rawYaw).plus(cameraYawOffset).minus(robotHeading);
//             return adjustedYawB.getDegrees();
//         }
//         return 0.0;
//     }

//     public boolean isLeftAlign(){
//         if (hasTarget()) {
//             return (rawYaw > VisionConstants.leftAlignRangeLeftInterval) && (rawYaw < VisionConstants.leftAlignRangeRightInterval);
//         }
//         else{
//             return false;
//         }
//     }

//     public double getTargetPitch() {
//         if (hasTarget()) {
//             return latestResult.getBestTarget().getPitch();
//         }
//         return 0.0;
//     }

//     public double getTargetArea() {
//         if (hasTarget()) {
//             return latestResult.getBestTarget().getArea();
//         }
//         return 0.0;
//     }

//     /**
//      * Estimate distance to target, accounting for camera height and pitch
//      * @param targetHeight Height of target from ground (meters)
//      * @return Distance in meters, -1 if no target
//      */
//     public double getDistanceToTarget(double targetHeight) {
//         if (hasTarget()) {
//             double pitch = getTargetPitch();
//             double totalPitch = Math.toRadians(cameraPitch + pitch);
//             return (targetHeight - cameraHeight) / Math.tan(totalPitch);
//         }
//         return -1.0;
//     }

//     public PhotonTrackedTarget getBestTarget() {
//         if (hasTarget()) {
//             return latestResult.getBestTarget();
//         }
//         return null;
//     }

//     public Translation2d getCameraOffset() {
//         return cameraOffset;
//     }

//     public Rotation2d getCameraYawOffset() {
//         return cameraYawOffset;
//     }
// }

/*----------------------------------Original Version-------------------------------------*/

// public class VisionSubsystem extends SubsystemBase {
//     private final PhotonCamera camera;
//     private PhotonPipelineResult latestResult;

//     private final String cameraName;

    
//     // Camera position relative to robot center (in meters)
//     private final Translation2d cameraOffset; // x (forward), y (left/right)
//     private final Rotation2d cameraYawOffset; // Camera's yaw relative to robot heading
//     private final double cameraHeight; // z (height) in meters
//     private final double cameraPitch; // Pitch angle in degrees

//     private double rawYaw;
//     // private double rawYawG;
//     private double target_x;
//     private double target_y;
//     private double target_z;

//     public VisionSubsystem() {

//         cameraName = "GSC_BLACK";

//         camera = new PhotonCamera(cameraName);

//         latestResult = new PhotonPipelineResult();
        
//         cameraOffset = new Translation2d(VisionConstants.cameraXOffset, VisionConstants.cameraYOffset);
//         cameraYawOffset = Rotation2d.fromDegrees(VisionConstants.cameraYawOffset);
//         cameraHeight = VisionConstants.cameraHeight;
//         cameraPitch = VisionConstants.cameraPitch;

//     }

//     @Override
//     public void periodic() {
//         // query camera
//         latestResult = camera.getLatestResult();
//         // latestResultG = cameraG.getLatestResult();
//         boolean hasTarget = latestResult.hasTargets();
//         // avoid null pointer exception if no tracked target
//         if ( hasTarget ) {
//             PhotonTrackedTarget target = latestResult.getBestTarget();
//             int targetID = target.getFiducialId();
//             double poseAmbiguity = target.getPoseAmbiguity();
//             Transform3d bestCameraToTargetPose = target.getBestCameraToTarget();

//             rawYaw = target.getYaw();
//             target_x = bestCameraToTargetPose.getX();
//             target_y = bestCameraToTargetPose.getY();
//             target_z = bestCameraToTargetPose.getZ();
//             // mpk - comment out after verifing target values
//             SmartDashboard.putNumber("targetID",targetID);
//             SmartDashboard.putNumber("Ambiguity",poseAmbiguity);
//             SmartDashboard.putNumber("cameraB Yaw", rawYaw);
//             SmartDashboard.putNumber("target_x", target_x);
//             SmartDashboard.putNumber("target_y", target_y);
//             SmartDashboard.putNumber("target_z", target_z);
//         }

//         // rawYawB = latestResultB.getBestTarget().getYaw();
//         // rawYawG = latestResultG.getBestTarget().getYaw();

//         // SmartDashboard.putNumber("cameraB Yaw", rawYawB);
//         // SmartDashboard.putNumber("camraG Yaw", rawYawG);

//         SmartDashboard.putBoolean("isLeftAligned", isLeftAlign());
//     }


//     public boolean hasTarget() {
//         return latestResult.hasTargets();      // mpk - should be periodic result via public gettter/setter interface?
//     }

//     public double getTargetYawAdjusted(Rotation2d robotHeading) {
//         if (hasTarget()) {
//             double rawYaw = latestResult.getBestTarget().getYaw();
//             // Adjust yaw for camera's orientation and position
//             Rotation2d adjustedYaw = Rotation2d.fromDegrees(rawYaw).plus(cameraYawOffset).minus(robotHeading);
//             return adjustedYaw.getDegrees();
//         }
//         return 0.0;
//     }

//     public boolean isLeftAlign(){
//         if (hasTarget()) {
//             return (rawYaw > VisionConstants.leftAlignRangeLeftInterval) && (rawYaw < VisionConstants.leftAlignRangeRightInterval);
//         }
//         else{
//             return false;
//         }
//     }

//     public double getTargetPitch() {
//         if (hasTarget()) {
//             return latestResult.getBestTarget().getPitch();
//         }
//         return 0.0;
//     }

//     public double getTargetArea() {
//         if (hasTarget()) {
//             return latestResult.getBestTarget().getArea();
//         }
//         return 0.0;
//     }

//     public double getDistanceToTarget(double targetHeight) {
//         if (hasTarget()) {
//             double pitch = getTargetPitch();
//             double totalPitch = Math.toRadians(cameraPitch + pitch);
//             return (targetHeight - cameraHeight) / Math.tan(totalPitch);
//         }
//         return -1.0;
//     }

//     public PhotonTrackedTarget getBestTarget() {
//         if (hasTarget()) {
//             return latestResult.getBestTarget();
//         }
//         return null;
//     }

//     public Translation2d getCameraOffset() {
//         return cameraOffset;
//     }

//     public Rotation2d getCameraYawOffset() {
//         return cameraYawOffset;
//     }
// }

/*----------------------------------Version 3------------------------------------- */

public class VisionSubsystem extends SubsystemBase {
    private final PhotonCamera camera;
    private PhotonPipelineResult latestResult;

    private final String cameraName = "GSC_BLACK";

    
    // Camera position relative to robot center (in meters)
    private final Translation2d cameraOffset;
    // x (forward), y (left/right)
    private final Rotation2d cameraYawOffset; // Camera's yaw relative to robot heading
    private final double cameraHeight; // z (height) in meters
    private final double cameraPitch; // Pitch angle in degrees

    private boolean hasTarget;
    private double poseAmbiguity;

    private PhotonTrackedTarget target;
    private int targetID;
    private Transform3d bestCameraToTargetPose;
    private double rawYaw;
    private double target_x;
    public static double target_y;
    private double target_z;


    public VisionSubsystem() {
        CommandScheduler.getInstance().registerSubsystem(this);
        
        camera = new PhotonCamera(cameraName);
        latestResult = new PhotonPipelineResult();
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
        hasTarget = latestResult.hasTargets();
        // avoid null pointer exception if no tracked target
        if ( hasTarget ) {
            target = latestResult.getBestTarget();
            targetID = target.getFiducialId();
            poseAmbiguity = target.getPoseAmbiguity();
            bestCameraToTargetPose = target.getBestCameraToTarget();

            rawYaw = target.getYaw();
            target_x = bestCameraToTargetPose.getX();
            target_y = bestCameraToTargetPose.getY();
            target_z = bestCameraToTargetPose.getZ();
            // mpk - comment out after verifing target values
            // SmartDashboard.putNumber("targetID",targetID);
            // SmartDashboard.putNumber("Ambiguity",poseAmbiguity);
            // SmartDashboard.putNumber("cameraB Yaw", rawYaw);
            // SmartDashboard.putNumber("target_x", target_x);
            // SmartDashboard.putNumber("target_y", target_y);
            // SmartDashboard.putNumber("target_z", target_z);
        }

        // rawYaw = latestResultB.getBestTarget().getYaw();

        // SmartDashboard.putNumber("camera Yaw", rawYawB);

        SmartDashboard.putBoolean("isLeftAligned", isLeftAlign());
    }


    public boolean hasTarget() {
        return hasTarget;      // mpk - should be periodic result via public gettter/setter interface?
    }

    public static double getTarget_y() {
        return target_y;
    }

    /**
     * Get yaw adjusted for camera offset
     * @param robotHeading Current robot heading (Rotation2d)
     * @return Yaw in degrees relative to robot frame
     */
    public double getTargetYawAdjusted(Rotation2d robotHeading) {
        if (hasTarget()) {
            double rawYaw = latestResult.getBestTarget().getYaw();
            // Adjust yaw for camera's orientation and position
            Rotation2d adjustedYawB = Rotation2d.fromDegrees(rawYaw).plus(cameraYawOffset).minus(robotHeading);
            return adjustedYawB.getDegrees();
        }
        return 0.0;
    }

    public boolean isLeftAlign(){
        if (hasTarget()) {
            return (rawYaw > VisionConstants.leftAlignRangeLeftInterval) && (rawYaw < VisionConstants.leftAlignRangeRightInterval);
        }
        else{
            return false;
        }
    }

    public double getTargetPitch() {
        if (hasTarget()) {
            return latestResult.getBestTarget().getPitch();
        }
        return 0.0;
    }

    public double getTargetArea() {
        if (hasTarget()) {
            return latestResult.getBestTarget().getArea();
        }
        return 0.0;
    }

    /**
     * Estimate distance to target, accounting for camera height and pitch
     * @param targetHeight Height of target from ground (meters)
     * @return Distance in meters, -1 if no target
     */
    public double getDistanceToTarget(double targetHeight) {
        if (hasTarget()) {
            double pitch = getTargetPitch();
            double totalPitch = Math.toRadians(cameraPitch + pitch);
            return (targetHeight - cameraHeight) / Math.tan(totalPitch);
        }
        return -1.0;
    }

    public PhotonTrackedTarget getBestTarget() {
        if (hasTarget()) {
            return target;
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