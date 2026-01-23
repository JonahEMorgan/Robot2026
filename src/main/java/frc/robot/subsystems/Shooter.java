package frc.robot.subsystems;

import frc.robot.Constants.Subsystems.ShooterConstants;
import frc.robot.utilities.Compliance;

/**
 * A subsystem which spins the shooter and launches fuel.
 */
public class Shooter extends BasicMotorSubsystem {
	/**
	 * Creates a new subsystem with a proper name
	 */
	public Shooter() {
		super();
		setName("Shooter subsystem");
	}

	/**
	 * Gets the CAN id from the shooter constants for the motor controller.
	 *
	 * @return CAN id
	 */
	@Override
	protected int getMotorId() {
		return Compliance.ensure(ShooterConstants.class, "kMotorPort");
	}

	/**
	 * Gets the default speed from the shooter constants for the motor.
	 *
	 * @return default speed
	 */
	@Override
	protected double getDefaultSpeed() {
		return Compliance.ensure(ShooterConstants.class, "kDefaultSpeed");
	}

}
