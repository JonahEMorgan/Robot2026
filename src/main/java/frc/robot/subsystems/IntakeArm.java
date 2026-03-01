package frc.robot.subsystems;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

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

	private static final SparkMaxConfig s_config = new SparkMaxConfig();

	{
		s_config.smartCurrentLimit(IntakeConstants.kArmSmartCurrentLimit);
		s_config.secondaryCurrentLimit(IntakeConstants.kArmSecondaryCurrentLimit);
		s_config.idleMode(IdleMode.kBrake);
		s_config.inverted(IntakeConstants.kArmInvert);
		s_config.encoder.positionConversionFactor(IntakeConstants.kArmConversionFactor);
	}

	public IntakeArm() {
		super(IntakeConstants.kIntakeArmPort, s_config);

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
