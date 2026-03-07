package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PositionControlSubsystem extends SubsystemBase {
	private final SparkMax m_motor;

	public PositionControlSubsystem(int port) {
		m_motor = new SparkMax(port, MotorType.kBrushless);
	}

	public void setMotorPosition(double position) {
		m_motor.getClosedLoopController().setSetpoint(position, ControlType.kPosition);
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

	public void configureMotor(SparkMaxConfig config) {
		m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber(getClass().getName() + "/Motor Position", getMotorRotations());
	}
}
