package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Subsystems.TurretConstants;

/**
 * A subsystem which has the ability to change both yaw and pitch in order to
 * adjust the direction fuel will exit the shooter.
 */
public class Turret extends SubsystemBase {
	/**
	 * The yaw motor allows the mechanism to pivot left and right.
	 */
	private final SparkMax m_motor;
	/**
	 * The through bore encoder on the yaw axis allows us to check the yaw angle.
	 */
	private final RelativeEncoder m_encoder;
	/**
	 * The through bore encoder on the pitch axis allows us to check the pitch
	 * angle.
	 * /**
	 * The yaw PID controller is used to aim the turret in a specific yaw direction
	 * in a closed feedback loop.
	 */
	// private final PIDController m_pid;

	/**
	 * The pitch PID controller is used to aim the turret in a specific pitch
	 * direction in a closed feedback loop.
	 */

	/**
	 * Creates a new subsystem using the motor id's from the turret constants, the
	 * corresponding pid constants from the turret constants, and a proper name.
	 */
	public Turret(int index) {
		m_motor = new SparkMax(index, MotorType.kBrushless);
		m_encoder = m_motor.getEncoder();
	}

	public void resetEncoder() {
		m_encoder.setPosition(0);
	}

	public double getPosition() {
		return m_encoder.getPosition();
	}

	public void runMotorAtDutyCycle(double dutyCycle) {
		double sign = Math.signum(dutyCycle);
		dutyCycle = Math.min(TurretConstants.maxDutyCycle, Math.abs(dutyCycle));

		m_motor.set(dutyCycle * sign);
	}
}
