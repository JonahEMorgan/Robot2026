package frc.robot.subsystems;

import frc.robot.Compliance;
import frc.robot.Constants.Subsystems.ClimberConstants;

public class Climber extends BasicMotorSubsystem {
	public Climber() {
		super();
		setName("Climber subsystem");
	}

	@Override
	protected int getMotorId() {
		return Compliance.ensure(ClimberConstants.class, "kMotorPort");
	}

	@Override
	protected double getDefaultSpeed() {
		return Compliance.ensure(ClimberConstants.class, "kDefaultSpeed");
	}

}
