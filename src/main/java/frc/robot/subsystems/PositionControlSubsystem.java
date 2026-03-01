package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Subsystems.IntakeConstants;

public class PositionControlSubsystem extends SubsystemBase {
	private static SparkMax m_motor;

	public PositionControlSubsystem(int port) {
		m_motor = new SparkMax(port, MotorType.kBrushless);
		// m_motor.getEncoder().setPosition(0);

		// TODO: Check configuration of motors
		SparkMaxConfig armConfig = new SparkMaxConfig();
		armConfig.smartCurrentLimit(IntakeConstants.kArmSmartCurrentLimit);
		armConfig.secondaryCurrentLimit(IntakeConstants.kArmSecondaryCurrentLimit);
		armConfig.idleMode(IdleMode.kBrake);
		armConfig.inverted(IntakeConstants.kArmInvert);
		armConfig.encoder.positionConversionFactor(IntakeConstants.kArmConversionFactor);

		m_motor.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
	}

	public static void setMotorPower(double power) {
		m_motor.set(power);
	}

	public static double getMotorRotations() {
		return m_motor.getEncoder().getPosition();
	}

	public static void stopMotor() {
		m_motor.stopMotor();
	}

	public static void resetMotorEncoder() {
		m_motor.getEncoder().setPosition(0);
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber(getClass().getName() + "/Motor Position", getMotorRotations());
	}
}
