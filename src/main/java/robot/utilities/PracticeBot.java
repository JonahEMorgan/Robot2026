package robot.utilities;

public class PracticeBot extends CompBot {
	/**
	 * Configures the motor controllers according to the CAN ids as well as the
	 * constants.
	 * 
	 * @param drivePort CAN id of drive motor
	 * @param steerPort CAN id of steer motor
	 */
	public PracticeBot(int drivePort, int steerPort) {
		super(drivePort, steerPort);
	}
}