package frc.robot;

import org.photonvision.PhotonCamera;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
	private final PhotonCamera m_camera;
	private final AprilTagFieldLayout m_aprilTagFieldLayout = AprilTagFieldLayout
			.loadField(AprilTagFields.k2026RebuiltAndymark);

	public VisionSubsystem() {
		m_camera = new PhotonCamera("TestCamLBord");
	}

	public void periodic() {
		var results = m_camera.getAllUnreadResults();
		for (var result : results) {
			var multiTagResult = result.getMultiTagResult();
			SmartDashboard.putBoolean("Vision/Has Results", result.hasTargets());
			if (multiTagResult.isPresent()) {
				Pose3d tagPose = m_aprilTagFieldLayout.getTagPose(4).get();
				Pose3d hubPose = new Pose3d(new Pose2d(tagPose.getX() - 23.5, tagPose.getY(),
						new Rotation2d()));
				Translation3d poseDiff = hubPose.getTranslation().minus(
						multiTagResult.get().estimatedPose.best.getTranslation());
				SmartDashboard.putNumber("Vision/PoseX", poseDiff.getX());
				SmartDashboard.putNumber("Vision/PoseY", poseDiff.getY());
				SmartDashboard.putNumber("Vision/PoseZ", poseDiff.getZ());
			}

			/*
			 * else if (result.hasTargets()) {
			 * PhotonTrackedTarget target = result.getBestTarget();
			 * Transform3d botPose = target.getBestCameraToTarget();
			 * SmartDashboard.putNumber("Vision/PoseX", botPose.getX());
			 * SmartDashboard.putNumber("Vision/PoseY", botPose.getY());
			 * SmartDashboard.putNumber("Vision/PoseZ", botPose.getZ());
			 * int id = target.getFiducialId();
			 * var tagPose = m_aprilTagFieldLayout.getTagPose(id);
			 * if (tagPose.isPresent()) {
			 * Pose3d botPose = PhotonUtils
			 * .estimateFieldToRobotAprilTag(
			 * target.getBestCameraToTarget(), tagPose.get(),
			 * new Transform3d(new Transform2d(0, 0, Rotation2d.fromDegrees(0))));
			 * SmartDashboard.putNumber("Vision/PoseX", botPose.getX());
			 * SmartDashboard.putNumber("Vision/PoseY", botPose.getY());
			 * SmartDashboard.putNumber("Vision/PoseZ", botPose.getZ());
			 * }
			 * }
			 */
		}
	}
}
