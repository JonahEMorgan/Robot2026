package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.CurrentUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.TorqueUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Force;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.units.measure.Resistance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Torque;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// This is really broken rn
public class PhysicalSwerveSim {
	private Angle m_turnAngle = Units.Radians.zero();
	private AngularVelocity m_turnVelocity = Units.RadiansPerSecond.zero();
	private Angle m_driveAngle = Units.Radians.zero();
	private AngularVelocity m_driveVelocity = Units.RadiansPerSecond.zero();

	private final Per<AngularVelocityUnit, VoltageUnit> m_turnKv;
	private final Per<TorqueUnit, CurrentUnit> m_turnKa;
	private final Resistance m_turnR;
	private final MomentOfInertia m_turnInertia;
	private final double m_turnRatio;
	private final Per<AngularVelocityUnit, VoltageUnit> m_driveKv;
	private final Per<TorqueUnit, CurrentUnit> m_driveKa;
	private final Resistance m_driveR;
	private final MomentOfInertia m_driveInertia;
	private final double m_driveRatio;
	private final LinearVelocity m_characteristicSpeed;
	private final Distance m_contactRadius;
	private final Force m_normalForce;
	private final Distance m_radius;
	private final double m_coefficientOfFriction;
	private final Torque m_driveTorque;

	public PhysicalSwerveSim() {
		this(DCMotor.getKrakenX44(1), DCMotor.getKrakenX60(1), 26, 675 / 112, 115, .74, 4, 2.25, 4, 500, 1, .01, .01);
	}

	public PhysicalSwerveSim(DCMotor turn, DCMotor drive, double turnRatio, double driveRatio, double robotWeight,
			double wheelWeight, double wheelDiameter, double wheelWidth, double wheelCount, double wheelStiffness,
			double coefficientOfFriction,
			double coefficientOfRollingResistance, double relaxationTime) {
		Distance width = Units.Inches.of(wheelWidth);
		Mass mass = Units.Pounds.of(wheelWeight);
		m_radius = Units.Inches.of(wheelDiameter).div(2);
		m_turnKv = Per.ofRelativeUnits(turn.KvRadPerSecPerVolt, Units.RadiansPerSecond.per(Units.Volt));
		m_turnKa = Per.ofRelativeUnits(turn.KtNMPerAmp, Units.NewtonMeters.per(Units.Amp));
		m_turnR = Units.Ohms.of(turn.rOhms);
		m_turnInertia = Units.KilogramSquareMeters.of(
				mass.in(Units.Kilograms)
						* (3 * Math.pow(m_radius.in(Units.Meters), 2) + Math.pow(width.in(Units.Meters), 2)) / 12);
		m_turnRatio = turnRatio;
		m_driveKv = Per.ofRelativeUnits(drive.KvRadPerSecPerVolt, Units.RadiansPerSecond.per(Units.Volt));
		m_driveKa = Per.ofRelativeUnits(drive.KtNMPerAmp, Units.NewtonMeters.per(Units.Amp));
		m_driveR = Units.Ohms.of(drive.rOhms);
		m_driveInertia = Units.KilogramSquareMeters
				.of(mass.in(Units.Kilograms) * Math.pow(m_radius.in(Units.Meters), 2) / 2);
		m_driveRatio = driveRatio;
		m_normalForce = Units.PoundForce.of(robotWeight).div(wheelCount);
		Measure<DistanceUnit> contactPatchLength = m_normalForce
				.divideRatio(Per.ofRelativeUnits(wheelStiffness, Units.PoundsForce.per(Units.Inch)));
		m_contactRadius = Units.Meters.of(Math.hypot(contactPatchLength.in(Units.Meters), width.in(Units.Meters)))
				.div(2);
		m_characteristicSpeed = Units.MetersPerSecond.of(contactPatchLength.in(Units.Meters) / relaxationTime);
		m_coefficientOfFriction = coefficientOfFriction;
		m_driveTorque = m_radius.times(m_normalForce.times(1 + coefficientOfRollingResistance));
	}

	private static Torque applyResistance(Torque output, Torque friction) {
		double outputValue = output.in(Units.NewtonMeters);
		double absValue = Math.abs(outputValue);
		double frictionValue = Math.abs(friction.in(Units.NewtonMeters));
		return Units.NewtonMeters.of(Math.copySign(Math.max(absValue - frictionValue, 0), outputValue));
	}

	private LinearVelocity getSpeed() {
		return Units.MetersPerSecond.of(m_driveVelocity.in(Units.RadiansPerSecond) * m_radius.in(Units.Meters));
	}

	public void simulate(double seconds, double turnVoltage, double driveVoltage) {
		simulate(seconds, turnVoltage, driveVoltage, true);
	}

	public void simulate(double seconds, double turnVoltage, double driveVoltage, boolean touchingGround) {
		Time time = Units.Seconds.of(seconds);
		{
			Torque resistance = m_contactRadius.times(
					2 / 3 * m_coefficientOfFriction * Math.exp(-getSpeed().div(m_characteristicSpeed).in(Units.Value)))
					.times(m_normalForce);
			Torque output = Units.NewtonMeters.of(
					Units.Volts.of(turnVoltage).minus(m_turnVelocity.divideRatio(m_turnKv))
							.div(m_turnR).timesRatio(m_turnKa).times(m_turnRatio).in(Units.NewtonMeters));
			Torque net = touchingGround ? applyResistance(output, resistance) : output;
			double acceleration = net.div(m_turnInertia).in(Units.NewtonMeters.per(Units.KilogramSquareMeters));
			m_turnVelocity = m_turnVelocity.plus(Units.RadiansPerSecondPerSecond.of(acceleration).times(time));
			m_turnAngle = m_turnAngle.plus(m_turnVelocity.times(time));
		}
		{
			Torque output = Units.NewtonMeters.of(
					Units.Volts.of(driveVoltage).minus(m_driveVelocity.divideRatio(m_driveKv))
							.div(m_driveR).timesRatio(m_driveKa).times(m_driveRatio).in(Units.NewtonMeters));
			SmartDashboard.putNumber("Output T (Nm)", output.in(Units.NewtonMeters));
			SmartDashboard.putNumber("Resistance T (Nm)", m_driveTorque.in(Units.NewtonMeters));
			Torque net = touchingGround ? applyResistance(output, m_driveTorque) : output;
			double acceleration = net.div(m_driveInertia).in(Units.NewtonMeters.per(Units.KilogramSquareMeters));
			m_driveVelocity = m_driveVelocity.plus(Units.RadiansPerSecondPerSecond.of(acceleration).times(time));
			m_driveAngle = m_driveAngle.plus(m_driveVelocity.times(time));
		}
	}

	public SwerveModuleState getModuleState() {
		return new SwerveModuleState(getSpeed(), Rotation2d.fromDegrees(m_turnAngle.in(Units.Degrees)));
	}
}
