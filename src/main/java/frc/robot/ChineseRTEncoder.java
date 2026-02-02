package frc.robot;

import com.revrobotics.AbsoluteEncoder;

import edu.wpi.first.math.geometry.Rotation2d;

public class ChineseRTEncoder {
	private final AbsoluteEncoder m_encoderA;
	private final double m_gearRatioA;
	private final AbsoluteEncoder m_encoderB;
	private final double m_gearRatioB;
	private double m_lastResult = 0;

	public ChineseRTEncoder(AbsoluteEncoder encoderA, double gearRatioA, AbsoluteEncoder encoderB, double gearRatioB) {
		m_encoderA = encoderA;
		m_gearRatioA = gearRatioA;
		m_encoderB = encoderB;
		m_gearRatioB = gearRatioB;
	}

	public Rotation2d getAngle() {
		double a = m_encoderA.getPosition() * 360;
		double b = m_encoderB.getPosition() * 360;
		m_lastResult = ChineseRTCalculator.getAngle(m_lastResult, a, m_gearRatioA, b, m_gearRatioB);
		return Rotation2d.fromDegrees(m_lastResult);
	}
}
