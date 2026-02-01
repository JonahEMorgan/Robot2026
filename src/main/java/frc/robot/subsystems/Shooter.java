package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Subsystems.ShooterConstants;

public class Shooter extends SubsystemBase {

	private final TalonFX m_TalonFX = new TalonFX(56);

	private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0);
	private final DutyCycleOut m_dutyCycleRequest = new DutyCycleOut(0);

	public Shooter() {
		Slot0Configs slot0 = new Slot0Configs();
		slot0.kP = 2.4;
		slot0.kI = 0;
		slot0.kD = 0.1;
		slot0.kV = 0.125;
		m_TalonFX.getConfigurator().apply(slot0);
	}

	public void runShooterAtRPS(double rps) {

		double trueRPS;

		if (rps > 0 && rps < ShooterConstants.kMIN_RPS)
			trueRPS = ShooterConstants.kMIN_RPS;
		else if (rps > ShooterConstants.kMAX_RPS)
			trueRPS = ShooterConstants.kMAX_RPS;
		else
			trueRPS = rps;

		m_TalonFX.setControl(
				m_velocityRequest.withVelocity(trueRPS));
	}

	public void stopShooter() {
		m_TalonFX.setControl(m_dutyCycleRequest.withOutput(0.0));
	}

	public void runShooterAtPower(double power) {

		double truePower;
		if (power < ShooterConstants.kMIN_POWER)
			truePower = ShooterConstants.kMIN_POWER;
		else if (power > ShooterConstants.kMAX_POWER)
			truePower = ShooterConstants.kMAX_POWER;
		else
			truePower = power;

		m_TalonFX.setControl(m_dutyCycleRequest.withOutput(truePower));
	}
}
