package frc.robot.utilities;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;

public abstract class PhysicalModule {
	public static interface ModuleCreator {
		public PhysicalModule create(int drivePort, int steerPort);
	}

	public static record Constants(double kTeleopMaxVoltage, double kTeleopMaxTurnVoltage, double kDriveGearRatio,
			double kSteerGearRatio, double kWheelDiameter) {
		public double getWheelCircumference() {
			return kWheelDiameter * Math.PI;
		}

		public double getMetersPerMotorRotation() {
			return kWheelDiameter * Math.PI / kDriveGearRatio;
		}
	}

	private double m_driveVoltage, m_steerVoltage, m_driveRotations;

	public abstract Constants getConstants();

	public abstract DCMotor getGearbox();

	public abstract void enableCoast();

	public abstract void enableBrake();

	public abstract double getDriveRotationsInternal();

	public abstract void setDriveRotationsInternal(double rotations);

	public abstract double getDriveCurrent();

	public abstract double getSteerCurrent();

	protected abstract double getDriveVoltageInternal();

	protected abstract double getSteerVoltageInternal();

	protected abstract void setDriveVoltageInternal(double voltage);

	protected abstract void setSteerVoltageInternal(double voltage);

	public <T> T simulationSwitch(T simulation, T reality) {
		return (switch (RobotBase.getRuntimeType()) {
			case kRoboRIO, kRoboRIO2 -> reality;
			case kSimulation -> simulation;
		});
	}

	public <T> T simulationSupplier(Supplier<T> simulation, Supplier<T> reality) {
		return simulationSwitch(simulation, reality).get();
	}

	public <T> void simulationConsumer(Consumer<T> simulation, Consumer<T> reality, T value) {
		simulationSwitch(simulation, reality).accept(value);
	}

	public double getDriveVoltage() {
		return simulationSupplier(() -> m_driveVoltage, this::getDriveVoltageInternal);
	}

	public double getSteerVoltage() {
		return simulationSupplier(() -> m_steerVoltage, this::getSteerVoltageInternal);
	}

	public void setDriveVoltage(double voltage) {
		simulationConsumer(v -> m_driveVoltage = v, this::setDriveVoltageInternal, voltage);
	}

	public void setSteerVoltage(double voltage) {
		simulationConsumer(v -> m_steerVoltage = v, this::setSteerVoltageInternal, voltage);
	}

	public double getDriveRotations() {
		return simulationSupplier(() -> m_driveRotations, this::getDriveRotationsInternal);
	};

	public void setDriveRotations(double rotations) {
		simulationConsumer(r -> m_driveRotations = r, this::setDriveRotationsInternal, rotations);
	};

	public void resetDriveRotations() {
		setDriveRotations(0);
	};

	public DCMotor getSteerGearbox() {
		return getGearbox();
	};

	public DCMotor getDriveGearbox() {
		return getGearbox();
	}
}