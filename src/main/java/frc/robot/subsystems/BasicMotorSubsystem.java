package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public abstract class BasicMotorSubsystem extends SubsystemBase {
	private final SparkMax m_motor;
	private final double m_speed;

	public BasicMotorSubsystem() {
		m_motor = new SparkMax(getMotorId(), MotorType.kBrushless);
		m_speed = getDefaultSpeed();
	}

	protected abstract int getMotorId();

	protected abstract double getDefaultSpeed();

	public Command moveForTime(double speed, double time) {
		return startEnd(
				() -> m_motor.set(speed),
				() -> m_motor.stopMotor())
						.withTimeout(time)
						.withName("moveForTime");
	}

	public Command moveForTime(double time) {
		return moveForTime(m_speed, time);
	}

	public Command moveWithTrigger(double speed, Trigger forward, Trigger back) {
		return runEnd(
				() -> {
					double sum = 0;
					if (forward != null) {
						if (forward.getAsBoolean()) {
							sum += speed;
						}
					}
					if (back != null) {
						if (back.getAsBoolean()) {
							sum -= speed;
						}
					}
					m_motor.set(sum);
				},
				() -> m_motor.stopMotor())
						.withName("moveWithTrigger");
	}

	public Command moveWithTrigger(Trigger forward, Trigger back) {
		return moveWithTrigger(m_speed, forward, back);
	}
}
