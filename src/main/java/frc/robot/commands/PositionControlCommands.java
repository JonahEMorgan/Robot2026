package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ClampedP;
import frc.robot.subsystems.PositionControlSubsystem;

public abstract class PositionControlCommands<T extends PositionControlSubsystem> {
	public class SpinMotorPowerForTime extends Command {
		private final double m_speed;
		private final double m_time;
		private final T m_subsystem;
		private final Timer m_timer = new Timer();

		public SpinMotorPowerForTime(T subsystem, double speed, double time) {
			setName("Spin at Power for Time");
			addRequirements(subsystem);
			m_speed = speed;
			m_time = time;
			m_subsystem = subsystem;
		}

		@Override
		public void initialize() {
			m_subsystem.setMotorPower(m_speed);
			m_timer.reset();
			m_timer.start();
		}

		@Override
		public void end(boolean interrupted) {
			m_subsystem.stopMotor();
			m_timer.stop();
		}

		@Override
		public boolean isFinished() {
			return m_time > 0 && m_timer.hasElapsed(m_time);
		}
	}

	public class MoveMotorToPosition extends Command {
		private final double m_position;
		private final T m_subsystem;
		private final boolean m_finite;
		private final double m_minPower;
		private final double m_maxPower;
		private final double m_maxError;
		private final double m_tolerance;

		// position is an absolute position, in number of rotations,
		// relative to the last time the subsytem was zeroed
		public MoveMotorToPosition(T subsystem, double position, double minPower,
				double maxPower, double maxError, double tolerance, boolean hold) {
			addRequirements(subsystem);
			m_position = position;
			m_subsystem = subsystem;
			m_minPower = minPower;
			m_maxPower = maxPower;
			m_maxError = maxError;
			m_tolerance = tolerance;
			m_finite = !hold;
		}

		@Override
		public void execute() {
			double error = m_subsystem.getMotorRotations() - m_position;
			m_subsystem.setMotorPower(ClampedP.clampedP(error, m_minPower, m_maxPower, m_maxError, m_tolerance));
		}

		@Override
		public void end(boolean interrupted) {
			m_subsystem.stopMotor();
		}

		@Override
		public boolean isFinished() {
			return m_finite && Math.abs(m_position - m_subsystem.getMotorRotations()) <= m_tolerance;
		}
	}

	public class ResetEncoder extends Command {
		private final T m_subsystem;

		public ResetEncoder(T subsystem) {
			m_subsystem = subsystem;
			setName(String.format("Reset encoder for %s", m_subsystem.getName()));
			addRequirements(m_subsystem);
		}

		@Override
		public void initialize() {
			m_subsystem.resetMotorEncoder();
		}

		@Override
		public boolean isFinished() {
			return true;
		}
	}
}
