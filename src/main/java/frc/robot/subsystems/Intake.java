package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Compliance;
import frc.robot.Constants.Subsystems.IntakeConstants;

public class Intake extends BasicMotorSubsystem {
	private final SparkMax m_deploy;
	private final PIDController m_pid;
	private final RelativeEncoder m_encoder;

	public Intake() {
		setName("Intake subsystem");
		int id = Compliance.ensure(IntakeConstants.class, "kDeployPort");
		m_deploy = new SparkMax(id, MotorType.kBrushless);
		m_encoder = m_deploy.getEncoder();
		double kP = Compliance.ensure(IntakeConstants.class, "kP");
		m_pid = new PIDController(kP, 0, 0);
	}

	private Command moveToAngleCommand(double angle) {
		return runEnd(
				() -> m_deploy.set(
						m_pid.calculate(
								m_encoder.getPosition())),
				() -> m_deploy.stopMotor())
						.beforeStarting(
								() -> m_pid.setSetpoint(angle))
						.until(
								() -> m_pid.atSetpoint())
						.withName("moveToAngleCommand");
	}

	public Command deployRollers() {
		double angle = Compliance.ensure(IntakeConstants.class, "kDeployAngle");
		return moveToAngleCommand(angle);
	}

	public Command retractRollers() {
		double angle = Compliance.ensure(IntakeConstants.class, "kRetractAngle");
		return moveToAngleCommand(angle);
	}

	@Override
	protected int getMotorId() {
		return Compliance.ensure(IntakeConstants.class, "kRollerPort");
	}

	@Override
	protected double getDefaultSpeed() {
		return Compliance.ensure(IntakeConstants.class, "kDefaultSpeed");
	}
}
