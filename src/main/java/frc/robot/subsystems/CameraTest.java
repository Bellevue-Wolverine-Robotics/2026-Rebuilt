package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CameraTest extends SubsystemBase {
    private final PhotonCamera camera;

    public CameraTest() {
        // Try to connect to a camera
        // This name should match what you set in PhotonVision
        // For testing, use any name - it will fail gracefully if not found
        camera = new PhotonCamera("test_camera");
    }

    @Override
    public void periodic() {
        // Check if camera is connected
        boolean isConnected = camera.isConnected();
        SmartDashboard.putBoolean("Camera/Connected", isConnected);

        if (!isConnected) {
            SmartDashboard.putString("Camera/Status", "NOT CONNECTED - Check PhotonVision");
            return;
        }

        // Camera is connected, get latest results
        // Changed from getLatestResult() to getAllUnreadResults()
        var results = camera.getAllUnreadResults();

        if (results.isEmpty()) {
            SmartDashboard.putString("Camera/Status", "Connected - No new results");
            return;
        }

        // Get the most recent result
        PhotonPipelineResult result = results.get(results.size() - 1);

        // Check if any targets are detected
        boolean hasTargets = result.hasTargets();
        SmartDashboard.putBoolean("Camera/Has Targets", hasTargets);

        if (hasTargets) {
            // Get the best target
            PhotonTrackedTarget target = result.getBestTarget();

            // Display target information
            SmartDashboard.putNumber("Target/Yaw", target.getYaw());
            SmartDashboard.putNumber("Target/Pitch", target.getPitch());
            SmartDashboard.putNumber("Target/Area", target.getArea());
            SmartDashboard.putNumber("Target/Skew", target.getSkew());

            // For AprilTags
            if (target.getFiducialId() != -1) {
                SmartDashboard.putNumber("Target/AprilTag ID", target.getFiducialId());
            }

            // Number of targets detected
            SmartDashboard.putNumber("Camera/Target Count", result.getTargets().size());

            SmartDashboard.putString("Camera/Status", "TRACKING " + result.getTargets().size() + " targets");
        } else {
            SmartDashboard.putString("Camera/Status", "Connected - No targets visible");
            SmartDashboard.putNumber("Camera/Target Count", 0);
        }

        // Latency information - API changed in 2025
        // getLatencyMillis() is now getLatency().in(Milliseconds)
        SmartDashboard.putNumber("Camera/Timestamp", result.getTimestampSeconds());

        // Alternative: Show number of unread results
        SmartDashboard.putNumber("Camera/Unread Results", results.size());
    }

    /**
     * Check if PhotonLib is working
     * @return true if library is installed correctly
     */
    public boolean isPhotoLibWorking() {
        try {
            // If we can create a camera object, PhotonLib is installed
            return true;
        } catch (Exception e) {
            System.err.println("PhotonLib Error: " + e.getMessage());
            return false;
        }
    }

    /**
     * Get camera connection status
     * @return true if camera is connected to NetworkTables
     */
    public boolean isCameraConnected() {
        return camera.isConnected();
    }
}