package frc.robot.subsystems;

import frc.robot.Compliance;
import frc.robot.Constants.Subsystems.TransportConstants;

public class Transport extends BasicMotorSubsystem {
	public Transport() {
		super();
		setName("Transport subsystem");
	}

	@Override
	protected int getMotorId() {
		return Compliance.ensure(TransportConstants.class, "kMotorPort");
	}

	@Override
	protected double getDefaultSpeed() {
		return Compliance.ensure(TransportConstants.class, "kDefaultSpeed");
	}

}
