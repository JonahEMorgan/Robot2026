package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

public class DistanceCalc {

	private AprilTagFieldLayout fieldLayout;

	public DistanceCalc(AprilTagFieldLayout fieldLayout) {
		this.fieldLayout = fieldLayout;
	}

	public double calculateDistanceToTag(int tagID, Pose2d robotPose) {
		// Get the pose of the AprilTag from the field layout
		Pose3d tagPose3d = fieldLayout.getTagPose(tagID).orElse(null);
		if (tagPose3d == null) {
			throw new IllegalArgumentException("Tag ID " + tagID + " not found in the field layout.");
		}

		// Convert Pose3d to Pose2d
		Pose2d tagPose = tagPose3d.toPose2d();

		// Calculate the transformation from the robot to the tag
		Transform2d transform = robotPose.minus(tagPose);

		// Get the translation component of the transform
		Translation2d translation = transform.getTranslation();

		// Calculate and return the distance
		return translation.getNorm();
	}
}
