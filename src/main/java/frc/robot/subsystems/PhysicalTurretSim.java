package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.CurrentUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.TorqueUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.units.measure.Resistance;
import edu.wpi.first.units.measure.Voltage;

public class PhysicalTurretSim {
	private Angle m_angle = Radians.zero();
	private AngularVelocity m_velocity = RadiansPerSecond.zero();

	private final Per<AngularVelocityUnit, VoltageUnit> m_kV;
	private final Per<TorqueUnit, CurrentUnit> m_kA;
	private final Resistance m_resistance;
	private final MomentOfInertia m_inertia;
	private final double m_gearRatio;

	public PhysicalTurretSim() {
		this(DCMotor.getNeo550(1), 200 / 36, 10, 6);
	}

	public PhysicalTurretSim(DCMotor motor, double gearRatio, double weight, double diameter) {
		m_kV = Per.ofRelativeUnits(motor.KvRadPerSecPerVolt, RadiansPerSecond.per(Volt));
		m_kA = Per.ofRelativeUnits(motor.KtNMPerAmp, NewtonMeters.per(Amp));
		m_resistance = Ohms.of(motor.rOhms);
		m_inertia = KilogramSquareMeters
				.of(Units.lbsToKilograms(weight) * Math.pow(Units.inchesToMeters(diameter) / 2, 2));
		m_gearRatio = gearRatio;
	}

	public void simulate(double seconds, double voltage) {
		Voltage potential = Volts.of(voltage).minus(m_velocity.divideRatio(m_kV));
		Measure<TorqueUnit> torque = potential.div(m_resistance).timesRatio(m_kA).times(m_gearRatio);
		AngularAcceleration acceleration = RadiansPerSecondPerSecond
				.of(torque.in(NewtonMeters) / m_inertia.in(KilogramSquareMeters));
		m_angle = m_angle.plus(m_velocity.times(Seconds.of(seconds)));
		m_velocity = m_velocity.plus(acceleration.times(Seconds.of(seconds)));
	}

	public Rotation2d getAngle() {
		return new Rotation2d(m_angle);
	}
}
