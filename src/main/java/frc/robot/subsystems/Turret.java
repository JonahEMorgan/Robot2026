package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Compliance;
import frc.robot.Constants.Subsystems.TurretConstants;

public class Turret extends SubsystemBase {
	private final SparkMax m_yawMotor;
	private final SparkMax m_pitchMotor;
	private final SparkAbsoluteEncoder m_yawEncoder;
	private final SparkAbsoluteEncoder m_pitchEncoder;
	private final PIDController m_yawPid;
	private final PIDController m_pitchPid;

	public Turret() {
		setName("Turret subsystem");
		{
			int id = Compliance.ensure(TurretConstants.Yaw.class, "kMotorPort");
			m_yawMotor = new SparkMax(id, MotorType.kBrushless);
			m_yawEncoder = m_yawMotor.getAbsoluteEncoder();
			int kP = Compliance.ensure(TurretConstants.Yaw.class, "kP");
			m_yawPid = new PIDController(kP, 0, 0);
		}
		{
			int id = Compliance.ensure(TurretConstants.Pitch.class, "kMotorPort");
			m_pitchMotor = new SparkMax(id, MotorType.kBrushless);
			m_pitchEncoder = m_pitchMotor.getAbsoluteEncoder();
			int kP = Compliance.ensure(TurretConstants.Pitch.class, "kP");
			m_pitchPid = new PIDController(kP, 0, 0);
		}
	}

	public DoubleSupplier addDeadzone(DoubleSupplier supplier, double deadzone) {
		return () -> MathUtil.applyDeadband(supplier.getAsDouble(), deadzone);
	}

	public Command aimWithJoystick(DoubleSupplier yaw, DoubleSupplier pitch) {
		double deadzone = Compliance.ensure(TurretConstants.class, "kDeadzone");
		DoubleSupplier yawValue = addDeadzone(yaw, deadzone);
		DoubleSupplier pitchValue = addDeadzone(pitch, deadzone);
		return runEnd(
				() -> {
					m_yawMotor.set(yawValue.getAsDouble());
					m_pitchMotor.set(pitchValue.getAsDouble());
				},
				() -> {
					m_yawMotor.stopMotor();
					m_pitchMotor.stopMotor();
				})
						.withName("aimWithJoystick");
	}

	public Command aimAtAngle(double yaw, double pitch) {
		return aimAtAngle(
				() -> yaw,
				() -> pitch).beforeStarting(
						() -> {
							m_yawPid.setSetpoint(yaw);
							m_pitchPid.setSetpoint(pitch);
						}).until(
								() -> m_yawPid.atSetpoint() && m_pitchPid.atSetpoint());
	}

	public Command aimAtAngle(DoubleSupplier yaw, DoubleSupplier pitch) {
		return runEnd(
				() -> {
					m_yawMotor.set(
							m_yawPid.calculate(
									m_yawEncoder.getPosition(),
									yaw.getAsDouble()));
					m_pitchMotor.set(
							m_pitchPid.calculate(
									m_pitchEncoder.getPosition(),
									pitch.getAsDouble()));
				},
				() -> {
					m_yawMotor.stopMotor();
					m_pitchMotor.stopMotor();
				})
						.withName("aimAtAngle");
	}
}
