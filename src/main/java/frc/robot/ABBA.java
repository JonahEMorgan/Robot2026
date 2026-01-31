package frc.robot;

import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.DriveConstants;

public class ABBA {
	{
		RobotController.setBrownoutVoltage(DriveConstants.kBrownoutVoltage);
	}

	public static double getMaxVoltageSmooth() {
		double voltage = RobotController.getBatteryVoltage();
		return Math.min(.1 * Math.pow(1.5, voltage), 12);
	}

	public static double getMaxVoltage() {
		double voltage = RobotController.getBatteryVoltage();
		if (voltage < 7) {
			return 1.5;
		} else if (voltage < 8) {
			return 2.5;
		} else if (voltage < 9) {
			return 5;
		} else if (voltage < 10) {
			return 9.5;
		} else {
			return 12;
		}
	}
}
