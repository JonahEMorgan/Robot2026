package frc.robot.subsystems;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.Subsystems.ClimberConstants;
import frc.robot.PositionEnumValue;

public class Climber extends PositionControlSubsystem {
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

	public static Climber s_theClimber;

	private static final SparkMaxConfig s_config = new SparkMaxConfig();

	{
		s_config.smartCurrentLimit(ClimberConstants.kSmartCurrentLimit);
		s_config.secondaryCurrentLimit(ClimberConstants.kSecondaryCurrentLimit);
		s_config.idleMode(IdleMode.kBrake);
		s_config.inverted(ClimberConstants.kInvert);
		s_config.encoder.positionConversionFactor(ClimberConstants.kGearRatio);
	}

	public Climber() {
		super(ClimberConstants.kClimberPort, s_config);

		if (s_theClimber == null) {
			s_theClimber = new Climber();
		} else {
			throw new Error("Intake Arm subsystem is already constructed.");
		}
	}

	public static Climber getClimber() {
		return s_theClimber;
	}
}