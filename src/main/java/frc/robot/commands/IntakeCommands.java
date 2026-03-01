package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ClampedP;
import frc.robot.PositionEnumValue;
import frc.robot.subsystems.IntakeArm;
import frc.robot.subsystems.IntakeWheels;

public class IntakeCommands {
	public static class SpinArmPowerForTime extends Command {
		private double m_speed;
		private double m_time;
		private Timer m_timer = new Timer();

		public SpinArmPowerForTime(double speed, double time) {
			setName("Spin at Power for Time");
			addRequirements(IntakeWheels.getIntake(), IntakeArm.getIntakeArm());
			m_speed = speed;
			m_time = time;
		}

		@Override
		public void initialize() {
			IntakeArm.setMotorPower(m_speed);
			m_timer.reset();
			m_timer.start();
		}

		@Override
		public void end(boolean interrupted) {
			IntakeArm.stopMotor();
		}

		@Override
		public boolean isFinished() {
			return m_timer.hasElapsed(m_time);
		}
	}

	public static class SpinArmPower extends Command {
		private double m_speed;

		public SpinArmPower(double speed) {
			setName("Spin at Power for Time");
			addRequirements(IntakeWheels.getIntake(), IntakeArm.getIntakeArm());
			m_speed = speed;
		}

		@Override
		public void initialize() {
			IntakeArm.setMotorPower(m_speed);
		}

		@Override
		public void end(boolean interrupted) {
			IntakeArm.stopMotor();
		}

		@Override
		public boolean isFinished() {
			return false;
		}
	}

	public static class MoveArmToPosition extends Command {
		double m_position;
		private final double kTolerance = .125; // 1/8 of a turn of the winch

		// position is an absolute position, in number of rotations,
		// relative to the last time the subsytem was zeroed
		public MoveArmToPosition(double position) {
			m_position = position;
		}

		@Override
		public void execute() {
			double error = IntakeArm.getMotorRotations() - m_position;
			double minPower = 0.1;
			double maxPower = 1;
			double maxError = 0.5;
			IntakeArm.setMotorPower(ClampedP.clampedP(error, minPower, maxPower, maxError, kTolerance));
		}

		@Override
		public void end(boolean interrupted) {
			IntakeArm.stopMotor();
		}

		@Override
		public boolean isFinished() {
			return Math.abs(m_position - IntakeArm.getMotorRotations()) <= kTolerance;
		}
	}

	public static class MoveArmToEEEEnum extends MoveArmToPosition {
		public MoveArmToEEEEnum(PositionEnumValue value) {
			super(value.getPosition());
		}
	}

	public static class SpinIntake extends Command {
		private final double m_speed;

		public SpinIntake(double speed) {
			m_speed = speed;
			setName("Spin Intake");
		}

		public void initialize() {
			IntakeWheels.setWheelPower(m_speed);
		}

		// Update this to use limit switch instead of getArmAngle method.
		public boolean isFinished() {
			return false;
		}

		public void end() {
			IntakeWheels.stopWheel();
		}
	}

	public static class ResetEncoder extends Command {
		@Override
		public void initialize() {
			IntakeArm.resetMotorEncoder();
		}
	}

	public static class StopIntake extends Command {
		@Override
		public void initialize() {
			IntakeWheels.stopWheel();
		}

		@Override
		public boolean isFinished() {
			return true;
		}
	}
}