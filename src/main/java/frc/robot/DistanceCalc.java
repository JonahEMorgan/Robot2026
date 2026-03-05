package frc.robot;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DistanceCalc extends SubsystemBase {

	private PhotonCamera m_camera = new PhotonCamera("TestCamLBord");
	Object m_robotOdometry;
	// Constants for distance calculation
	private static final double CAMERA_HEIGHT_METERS = 0.53; // actual camera height
	private static final double TARGET_HEIGHT_METERS = 1.12; // tag 4 actual target height
	private static final double CAMERA_PITCH_RADIANS = Math.toRadians(20); // actual camera pitch angle
	private double targetPos = 0.0;
	private boolean targetVisible = false;
	private boolean authTarg = false;

	// Method to calculate distance to the target
	private double calculateDistanceToTargetMeters() {
		var results = m_camera.getLatestResult();
		PhotonTrackedTarget bestTarget = results.hasTargets() ? results.getBestTarget() : null;
		if (bestTarget != null) {
			double targetPitch = bestTarget.getPitch();
			return PhotonUtils.calculateDistanceToTargetMeters(
					CAMERA_HEIGHT_METERS,
					TARGET_HEIGHT_METERS,
					CAMERA_PITCH_RADIANS,
					Math.toRadians(targetPitch));
		} else {
			return -1; // Return -1 if no valid target is found
		}
	}

	public void init() {
		SmartDashboard.putNumber("distance (M)", targetPos * 39.37);
		SmartDashboard.getBoolean("hasTarget", targetVisible);
		SmartDashboard.putBoolean("authTag", authTarg);
	}

	public void periodic() {
		SmartDashboard.updateValues();
	}

	// Add debug statements and dynamic SmartDashboard updates
	public void processCameraResults() {
		System.out.println("Processing camera results...");
		// Read in relevant data from the Camera
		var results = m_camera.getAllUnreadResults();
		if (!results.isEmpty()) {
			System.out.println("New camera results available.");
			// Camera processed a new frame since last
			// Get the last one in the list.
			var result = results.get(results.size() - 1);
			if (result.hasTargets()) {
				System.out.println("Targets detected.");
				// At least one AprilTag was seen by the camera
				for (var target : result.getTargets()) {
					if (target.getFiducialId() == 10) {
						// Found Tag 4, record its information
						targetPos = calculateDistanceToTargetMeters();
						targetVisible = true;
						authTarg = true;
						System.out.println("Tag 4 detected. Distance: " + targetPos);
					} else {
						targetVisible = true;
						authTarg = false;
					}
				}
			} else {
				System.out.println("No targets detected.");
				targetVisible = false;
			}
		} else {
			targetVisible = false;
			targetPos = calculateDistanceToTargetMeters();
			System.out.println("No new results. Calculated distance: " + targetPos);
		}

		// Update SmartDashboard dynamically
		SmartDashboard.putNumber("distance (M)", targetPos * 39.37);
		SmartDashboard.putBoolean("hasTarget", targetVisible);
		SmartDashboard.putBoolean("authTag", authTarg);
	}

}
