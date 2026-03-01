package frc.robot.subsystems;

import frc.robot.Constants.Subsystems.IntakeConstants;
import frc.robot.PositionEnumValue;

public class IntakeArm extends PositionControlSubsystem {

	public enum Positions implements PositionEnumValue {
		Out(8), In(0);

		private final double m_position;

		private Positions(double position) {
			m_position = position;
		}

		@Override
		public double getPosition() {
			return m_position;
		}
	}

	public static IntakeArm s_theIntakeArm;

	public IntakeArm() {
		super(IntakeConstants.kIntakeArmPort);

		if (s_theIntakeArm == null) {
			s_theIntakeArm = new IntakeArm();
		} else {
			throw new Error("Intake Arm subsystem is already constructed.");
		}
	}

	public static IntakeArm getIntakeArm() {
		return s_theIntakeArm;
	}
}
