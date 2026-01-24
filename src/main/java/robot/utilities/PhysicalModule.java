package robot.utilities;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * A class which allows having multiple different types of swerve drive
 * hardware. This makes it easy to switch between different robots.
 */
public abstract class PhysicalModule {
	public static interface ModuleCreator {
		public PhysicalModule create(int drivePort, int steerPort);
	}

	public static record Constants(
			/**
			 * Stores the maximum voltage for drive motors.
			 */
			double kTeleopMaxVoltage,
			/**
			 * Stores the maximum voltage for steer motors.
			 */
			double kTeleopMaxTurnVoltage,
			/**
			 * Stores the gear ratio for drive motors.
			 */
			double kDriveGearRatio,
			/**
			 * Stores the gear ratio for steer motors.
			 */
			double kSteerGearRatio,
			/**
			 * Stores the wheel diameter in meters.
			 */
			double kWheelDiameter) {

		/**
		 * Gets the wheel circumference in meters.
		 * 
		 * @return wheel circumference in meters
		 */
		public double getWheelCircumference() {
			return kWheelDiameter * Math.PI;
		}

		/**
		 * Gets the distance the robot travels for each rotation of the wheels.
		 * 
		 * @return ratio of meters per each rotation
		 */
		public double getMetersPerMotorRotation() {
			return kWheelDiameter * Math.PI / kDriveGearRatio;
		}
	}

	/**
	 * The drive voltage, steer voltage, and number of drive rotations are stored
	 * for simulation purposes.
	 */
	private double m_driveVoltage, m_steerVoltage, m_driveRotations;

	/**
	 * Gets the constants specific to this physical robot.
	 * 
	 * @return constants
	 */
	public abstract Constants getConstants();

	/**
	 * Gets a gearbox in order to simulate the both the drive and steer motors.
	 * 
	 * @return gearbox
	 */
	public abstract DCMotor getGearbox();

	/**
	 * Enables coast mode on the underlying drive motor controller.
	 */
	public abstract void enableCoast();

	/**
	 * Enables brake mode on the underlying drive motor controller.
	 */
	public abstract void enableBrake();

	/**
	 * Gets the number of rotations made by the drive wheel.
	 * 
	 * @return distance in rotations
	 */
	public abstract double getDriveRotationsInternal();

	/**
	 * Sets the number of rotations made by the drive wheel
	 * 
	 * @param rotations distance in rotations
	 */
	public abstract void setDriveRotationsInternal(double rotations);

	/**
	 * Gets the current being used to power the drive motor.
	 * 
	 * @return current in amperes
	 */
	public abstract double getDriveCurrent();

	/**
	 * Gets the current being used to power the steer motor.
	 * 
	 * @return current in amperes
	 */
	public abstract double getSteerCurrent();

	/**
	 * Gets the voltage being used to power the drive motor.
	 * 
	 * @return voltage in volts
	 */
	protected abstract double getDriveVoltageInternal();

	/**
	 * Gets the voltage being used to power the steer motor.
	 * 
	 * @return voltage in volts
	 */
	protected abstract double getSteerVoltageInternal();

	/**
	 * Sets the voltage being used to power the drive motor.
	 * 
	 * @param voltage voltage in volts
	 */
	protected abstract void setDriveVoltageInternal(double voltage);

	/**
	 * Sets the voltage being used to power the steer motor.
	 * 
	 * @param voltage voltage in volts
	 */
	protected abstract void setSteerVoltageInternal(double voltage);

	/**
	 * Chooses between two values depending on whether we are in simulation or not.
	 * 
	 * @param <T> value type
	 * @param simulation value to choose during simulations
	 * @param reality value to chose in reality
	 * @return chosen value
	 */
	public <T> T simulationSwitch(T simulation, T reality) {
		return (switch (RobotBase.getRuntimeType()) {
			case kRoboRIO, kRoboRIO2 -> reality;
			case kSimulation -> simulation;
		});
	}

	/**
	 * Gets a value from a supplier depending on whether we are in simulation or
	 * not.
	 * 
	 * @param <T> supplier return type
	 * @param simulation supplier to evaluate during simulations
	 * @param reality supplier to evaluate in reality
	 * @return resulting value
	 */
	public <T> T simulationSupplier(Supplier<T> simulation, Supplier<T> reality) {
		return simulationSwitch(simulation, reality).get();
	}

	/**
	 * Sends a value to a consumer depending on whether we are in simulation or not.
	 * 
	 * @param <T> consumer input type
	 * @param simulation consumer to satisfy during simulations
	 * @param reality consumer to satisfy in reality
	 * @param value value to send to consumers
	 */
	public <T> void simulationConsumer(Consumer<T> simulation, Consumer<T> reality, T value) {
		simulationSwitch(simulation, reality).accept(value);
	}

	/**
	 * Gets the voltage being used to power the drive motor regardless of whether we
	 * are currently in simulation or not.
	 * 
	 * @return voltage in volts
	 */
	public double getDriveVoltage() {
		return simulationSupplier(() -> m_driveVoltage, this::getDriveVoltageInternal);
	}

	/**
	 * Gets the voltage being used to power the steer motor regardless of whether we
	 * are currently in simulation or not.
	 * 
	 * @return voltage in volts
	 */
	public double getSteerVoltage() {
		return simulationSupplier(() -> m_steerVoltage, this::getSteerVoltageInternal);
	}

	/**
	 * Sets the voltage being used to power the drive motor regardless of whether we
	 * are currently in simulation or not.
	 * 
	 * @param voltage voltage in volts
	 */
	public void setDriveVoltage(double voltage) {
		simulationConsumer(v -> m_driveVoltage = v, this::setDriveVoltageInternal, voltage);
	}

	/**
	 * Sets the voltage being used to power the steer motor regardless of whether we
	 * are currently in simulation or not.
	 * 
	 * @param voltage voltage in volts
	 */
	public void setSteerVoltage(double voltage) {
		simulationConsumer(v -> m_steerVoltage = v, this::setSteerVoltageInternal, voltage);
	}

	/**
	 * Gets the number of rotations made by the drive wheel regardless of whether we
	 * are currently in simulation or not.
	 * 
	 * @return distance in rotations
	 */
	public double getDriveRotations() {
		return simulationSupplier(() -> m_driveRotations, this::getDriveRotationsInternal);
	};

	/**
	 * Sets the number of rotations made by the drive wheel regardless of whether we
	 * are currently in simulation or not.
	 * 
	 * @param rotations distance in rotations
	 */
	public void setDriveRotations(double rotations) {
		simulationConsumer(r -> m_driveRotations = r, this::setDriveRotationsInternal, rotations);
	};

	/**
	 * Resets the number of rotations made by the drive wheel to zero.
	 */
	public void resetDriveRotations() {
		setDriveRotations(0);
	};

	/**
	 * Gets a gearbox in order to simulate the steer motor.
	 * 
	 * @return gearbox
	 */
	public DCMotor getSteerGearbox() {
		return getGearbox();
	};

	/**
	 * Gets a gearbox in order to simulate the drive motor.
	 * 
	 * @return gearbox
	 */
	public DCMotor getDriveGearbox() {
		return getGearbox();
	}
}