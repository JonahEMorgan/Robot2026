package robot.utilities;

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
	private final SparkMax m_driveMotor, m_steerMotor;

	public Constants getConstants() {
		return new Constants(12, 7.2, 6.75, 150. / 7, Units.inchesToMeters(4));
	}

	public Dennis(int drivePort, int steerPort) {
		m_driveMotor = new SparkMax(drivePort, MotorType.kBrushless);
		m_steerMotor = new SparkMax(steerPort, MotorType.kBrushless);
	}

	public DCMotor getGearbox() {
		return DCMotor.getNEO(1);
	}

	public void enableCoast() {
		m_driveMotor.configure(
				new SparkMaxConfig().idleMode(IdleMode.kCoast), ResetMode.kNoResetSafeParameters,
				PersistMode.kNoPersistParameters);
	}

	public void enableBrake() {
		m_driveMotor.configure(
				new SparkMaxConfig().idleMode(IdleMode.kBrake), ResetMode.kNoResetSafeParameters,
				PersistMode.kNoPersistParameters);
	}

	public double getDriveRotationsInternal() {
		return m_driveMotor.getEncoder().getPosition();
	}

	public void setDriveRotationsInternal(double rotations) {
		m_driveMotor.getEncoder().setPosition(rotations);
	}

	public double getDriveCurrent() {
		return m_driveMotor.getOutputCurrent();
	}

	public double getSteerCurrent() {
		return m_steerMotor.getOutputCurrent();
	}

	protected double getDriveVoltageInternal() {
		return m_driveMotor.getAppliedOutput() * m_driveMotor.getBusVoltage();
	}

	protected double getSteerVoltageInternal() {
		return m_steerMotor.getAppliedOutput() * m_steerMotor.getBusVoltage();
	}

	protected void setDriveVoltageInternal(double voltage) {
		m_driveMotor.setVoltage(voltage);
	}

	protected void setSteerVoltageInternal(double voltage) {
		m_steerMotor.setVoltage(voltage);
	}
}