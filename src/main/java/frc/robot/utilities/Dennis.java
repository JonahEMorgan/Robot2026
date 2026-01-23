package frc.robot.utilities;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

/**
 * A class for dennis, who uses neo motors. Dennis will be forever remembered as
 * the little bot that could.
 */
public class Dennis extends PhysicalModule {
	/**
	 * The drive motor uses a spark max motor controller.
	 */
	private final SparkMax m_driveMotor;
	/**
	 * The steer motor uses a spark max motor controller.
	 */
	private final SparkMax m_steerMotor;

	/**
	 * Gets the constants specific to Dennis.
	 * 
	 * @return constants
	 */
	@Override
	public Constants getConstants() {
		return new Constants(
				12,
				7.2,
				6.75,
				150. / 7,
				Units.inchesToMeters(4));
	}

	/**
	 * Configures the motor controllers according to the CAN ids.
	 * 
	 * @param drivePort CAN id of drive motor
	 * @param steerPort CAN id of steer motor
	 */
	public Dennis(int drivePort, int steerPort) {
		m_driveMotor = new SparkMax(drivePort, MotorType.kBrushless);
		m_steerMotor = new SparkMax(steerPort, MotorType.kBrushless);
	}

	/**
	 * Gets a neo gearbox for the steer motor.
	 * 
	 * @return gearbox
	 */
	@Override
	public DCMotor getSteerGearbox() {
		return DCMotor.getNEO(1);
	}

	/**
	 * Gets a neo gearbox for the drive motor.
	 * 
	 * @return gearbox
	 */
	@Override
	public DCMotor getDriveGearbox() {
		return DCMotor.getNEO(1);
	}

	/**
	 * Enables coast mode by changing the motor controller configuration.
	 */
	@Override
	public void enableCoast() {
		m_driveMotor.configure(
				new SparkMaxConfig()
						.idleMode(IdleMode.kCoast),
				ResetMode.kNoResetSafeParameters,
				PersistMode.kNoPersistParameters);
	}

	/**
	 * Enables brake mode by changing the motor controller configuration.
	 */
	@Override
	public void enableBrake() {
		m_driveMotor.configure(
				new SparkMaxConfig()
						.idleMode(IdleMode.kBrake),
				ResetMode.kNoResetSafeParameters,
				PersistMode.kNoPersistParameters);
	}

	/**
	 * Gets the number of rotations made by the drive wheel by polling the relative
	 * encoder.
	 * 
	 * @return distance in rotations
	 */
	@Override
	public double getDriveRotationsInternal() {
		return m_driveMotor.getEncoder().getPosition();
	}

	/**
	 * Sets the number of rotations made by the drive wheel by changing the value of
	 * the relative encoder.
	 * 
	 * @param rotations distance in rotations
	 */
	@Override
	public void setDriveRotationsInternal(double rotations) {
		m_driveMotor.getEncoder().setPosition(rotations);
	}

	/**
	 * Gets the current being used to power the drive motor.
	 * 
	 * @return current in amperes
	 */
	@Override
	public double getDriveCurrent() {
		return m_driveMotor.getOutputCurrent();
	}

	/**
	 * Gets the current being used to power the steer motor.
	 * 
	 * @return current in amperes
	 */
	@Override
	public double getSteerCurrent() {
		return m_steerMotor.getOutputCurrent();
	}

	/**
	 * Gets the voltage being used to power the drive motor.
	 * 
	 * @return voltage in volts
	 */
	@Override
	protected double getDriveVoltageInternal() {
		return m_driveMotor.getAppliedOutput() * m_driveMotor.getBusVoltage();
	}

	/**
	 * Gets the voltage being used to power the steer motor.
	 * 
	 * @return voltage in volts
	 */
	@Override
	protected double getSteerVoltageInternal() {
		return m_steerMotor.getAppliedOutput() * m_steerMotor.getBusVoltage();
	}

	/**
	 * Sets the voltage being used to power the drive motor.
	 * 
	 * @param voltage voltage in volts
	 */
	@Override
	protected void setDriveVoltageInternal(double voltage) {
		m_driveMotor.setVoltage(voltage);
	}

	/**
	 * Sets the voltage being used to power the steer motor.
	 * 
	 * @param voltage voltage in volts
	 */
	@Override
	protected void setSteerVoltageInternal(double voltage) {
		m_steerMotor.setVoltage(voltage);
	}

}
