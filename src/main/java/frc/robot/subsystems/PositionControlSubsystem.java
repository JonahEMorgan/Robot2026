package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PositionControlSubsystem extends SubsystemBase {
	private final SparkMax m_motor;

	public PositionControlSubsystem(int port, SparkMaxConfig config) {
		(m_motor = new SparkMax(port, MotorType.kBrushless))
				.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
	}

	public void setMotorPower(double power) {
		m_motor.set(power);
	}

	public double getMotorRotations() {
		return m_motor.getEncoder().getPosition();
	}

	public void stopMotor() {
		m_motor.stopMotor();
	}

	public void resetMotorEncoder() {
		m_motor.getEncoder().setPosition(0);
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber(getClass().getName() + "/Motor Position", getMotorRotations());
	}
}
