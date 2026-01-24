package robot.utilities;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import robot.Constants.DriveConstants;

public class CompBot extends PhysicalModule {
	private static final TalonFXConfiguration s_config = new TalonFXConfiguration();
	/**
	 * The drive and steer motors both use talon fx motor controllers.
	 */
	private final TalonFX m_driveMotor, m_steerMotor;

	/**
	 * Gets the constants specific to the comp bot.
	 * 
	 * @return constants
	 */
	@Override
	public Constants getConstants() {
		return new Constants(12, 7.2, 6.75, 150. / 7, Units.inchesToMeters(4));
	}

	/**
	 * Configures the motor controllers according to the CAN ids as well as the
	 * constants.
	 * 
	 * @param drivePort CAN id of drive motor
	 * @param steerPort CAN id of steer motor
	 */
	public CompBot(int drivePort, int steerPort) {
		(m_driveMotor = new TalonFX(drivePort)).getConfigurator().apply(DriveConstants.kDriveConfig);
		(m_steerMotor = new TalonFX(steerPort)).getConfigurator().apply(DriveConstants.kSteerConfig);
	}

	/**
	 * Gets a kraken gearbox for both the drive and steer motors.
	 * 
	 * @return gearbox
	 */
	@Override
	public DCMotor getGearbox() {
		return DCMotor.getKrakenX60(1);
	}

	/**
	 * Enables coast mode by changing the motor controller configuration.
	 */
	@Override
	public void enableCoast() {
		s_config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
		m_driveMotor.getConfigurator().apply(s_config);
	}

	/**
	 * Enables brake mode by changing the motor controller configuration.
	 */
	@Override
	public void enableBrake() {
		s_config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		m_driveMotor.getConfigurator().apply(s_config);
	}

	/**
	 * Gets the number of rotations made by the drive wheel by polling the relative
	 * encoder.
	 * 
	 * @return distance in rotations
	 */
	@Override
	public double getDriveRotationsInternal() {
		return m_driveMotor.getPosition().getValueAsDouble();
	}

	/**
	 * Sets the number of rotations made by the drive wheel by changing the value of
	 * the relative encoder.
	 * 
	 * @param rotations distance in rotations
	 */
	@Override
	public void setDriveRotationsInternal(double rotations) {
		m_driveMotor.setPosition(rotations);
	}

	/**
	 * Gets the current being used to power the drive motor.
	 * 
	 * @return current in amperes
	 */
	@Override
	public double getDriveCurrent() {
		return m_driveMotor.getStatorCurrent().getValueAsDouble();
	}

	/**
	 * Gets the current being used to power the steer motor.
	 * 
	 * @return current in amperes
	 */
	@Override
	public double getSteerCurrent() {
		return m_steerMotor.getStatorCurrent().getValueAsDouble();
	}

	/**
	 * Gets the voltage being used to power the drive motor.
	 * 
	 * @return voltage in volts
	 */
	@Override
	public double getDriveVoltageInternal() {
		return m_driveMotor.getMotorVoltage().getValueAsDouble();
	}

	/**
	 * Gets the voltage being used to power the steer motor.
	 * 
	 * @return voltage in volts
	 */
	@Override
	public double getSteerVoltageInternal() {
		return m_steerMotor.getMotorVoltage().getValueAsDouble();
	}

	/**
	 * Sets the voltage being used to power the drive motor.
	 * 
	 * @param voltage voltage in volts
	 */
	@Override
	public void setDriveVoltageInternal(double voltage) {
		m_driveMotor.setVoltage(voltage);
	}

	/**
	 * Sets the voltage being used to power the steer motor.
	 * 
	 * @param voltage voltage in volts
	 */
	@Override
	public void setSteerVoltageInternal(double voltage) {
		m_steerMotor.setVoltage(voltage);
	}
}