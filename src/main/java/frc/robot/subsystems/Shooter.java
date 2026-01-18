package frc.robot.subsystems;

import frc.robot.Compliance;
import frc.robot.Constants.Subsystems.ShooterConstants;

public class Shooter extends BasicMotorSubsystem {
	public Shooter() {
		super();
		setName("Shooter subsystem");
	}

	@Override
	protected int getMotorId() {
		return Compliance.ensure(ShooterConstants.class, "kMotorPort");
	}

	@Override
	protected double getDefaultSpeed() {
		return Compliance.ensure(ShooterConstants.class, "kDefaultSpeed");
	}

}
