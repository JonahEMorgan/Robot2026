package frc.robot.utilities;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.DriveConstants;

public class CompBot extends PhysicalModule {
	private static final TalonFXConfiguration s_config = new TalonFXConfiguration();
	private final TalonFX m_driveMotor, m_steerMotor;

	public Constants getConstants() {
		return new Constants(12, 7.2, 6.75, 150. / 7, Units.inchesToMeters(4));
	}

	public CompBot(int drivePort, int steerPort) {
		(m_driveMotor = new TalonFX(drivePort)).getConfigurator().apply(DriveConstants.kDriveConfig);
		(m_steerMotor = new TalonFX(steerPort)).getConfigurator().apply(DriveConstants.kSteerConfig);
	}

	public DCMotor getGearbox() {
		return DCMotor.getKrakenX60(1);
	}

	public void enableCoast() {
		s_config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
		m_driveMotor.getConfigurator().apply(s_config);
	}

	public void enableBrake() {
		s_config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		m_driveMotor.getConfigurator().apply(s_config);
	}

	public double getDriveRotationsInternal() {
		return m_driveMotor.getPosition().getValueAsDouble();
	}

	public void setDriveRotationsInternal(double rotations) {
		m_driveMotor.setPosition(rotations);
	}

	public double getDriveCurrent() {
		return m_driveMotor.getStatorCurrent().getValueAsDouble();
	}

	public double getSteerCurrent() {
		return m_steerMotor.getStatorCurrent().getValueAsDouble();
	}

	public double getDriveVoltageInternal() {
		return m_driveMotor.getMotorVoltage().getValueAsDouble();
	}

	public double getSteerVoltageInternal() {
		return m_steerMotor.getMotorVoltage().getValueAsDouble();
	}

	public void setDriveVoltageInternal(double voltage) {
		m_driveMotor.setVoltage(voltage);
	}

	public void setSteerVoltageInternal(double voltage) {
		m_steerMotor.setVoltage(voltage);
	}
}