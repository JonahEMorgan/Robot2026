package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ClampedP;
import frc.robot.Constants.Subsystems.TurretConstants;
import frc.robot.subsystems.Turret;

public class TurretCommand {
	private final Turret m_turretSubsystem;

	public TurretCommand(Turret turretSubsystem) {
		m_turretSubsystem = turretSubsystem;
	}

	public class RunToAngleHardware extends Command {

		private final double m_angle;

		/** Creates a new RunTurretAtSpeed. */
		public RunToAngleHardware(double angle) {
			addRequirements(m_turretSubsystem);
			m_angle = angle;
		}

		// Called every time the scheduler runs while the command is scheduled.
		@Override
		public void execute() {
			SmartDashboard.putNumber("Position", m_turretSubsystem.getPosition());
			m_turretSubsystem.setAngle(m_angle);
		}

		// Called once the command ends or is interrupted.
		@Override
		public void end(boolean interrupted) {
			m_turretSubsystem.runMotorAtDutyCycle(0);
		}

		// Returns true when the command should end.
		@Override
		public boolean isFinished() {
			return Math.abs(m_turretSubsystem.getPosition() - m_angle) < TurretConstants.kTolerance;
		}
	}

	public class RunToAngleHardwareSignal extends Command {

		private final DoubleSupplier m_x;
		private final DoubleSupplier m_y;

		/** Creates a new RunTurretAtSpeed. */
		public RunToAngleHardwareSignal(DoubleSupplier x, DoubleSupplier y) {
			addRequirements(m_turretSubsystem);
			m_x = x;
			m_y = y;
		}

		// Called every time the scheduler runs while the command is scheduled.
		@Override
		public void execute() {
			SmartDashboard.putNumber("Position", m_turretSubsystem.getPosition());
			double x = m_x.getAsDouble();
			double y = m_y.getAsDouble();
			if (Math.hypot(x, y) > 0.5) {
				double angle = Units.radiansToDegrees(Math.atan2(y, x));
				m_turretSubsystem.setAngle(angle);
			}
		}

		// Called once the command ends or is interrupted.
		@Override
		public void end(boolean interrupted) {
			m_turretSubsystem.runMotorAtDutyCycle(0);
		}

		// Returns true when the command should end.
		@Override
		public boolean isFinished() {
			return false;
		}
	}

	public class RunToAngle extends Command {
		private final double m_angle;

		/** Creates a new RunTurretAtSpeed. */
		public RunToAngle(double angle) {
			addRequirements(m_turretSubsystem);
			m_angle = angle;
		}

		// Called every time the scheduler runs while the command is scheduled.
		@Override
		public void execute() {
			SmartDashboard.putNumber("Position", m_turretSubsystem.getPosition());
			double error = m_turretSubsystem.getPosition() - m_angle;
			double power = ClampedP.clampedP(
					error, TurretConstants.kMinPower, TurretConstants.kMaxPower, TurretConstants.kMaxErr,
					TurretConstants.kTolerance);
			SmartDashboard.putNumber("Power", power);
			m_turretSubsystem.runMotorAtDutyCycle(power);
		}

		// Called once the command ends or is interrupted.
		@Override
		public void end(boolean interrupted) {
			m_turretSubsystem.runMotorAtDutyCycle(0);
		}

		// Returns true when the command should end.
		@Override
		public boolean isFinished() {
			return Math.abs(m_turretSubsystem.getPosition() - m_angle) < TurretConstants.kTolerance;
		}
	}

	/*
	 * You should consider using the more terse Command factories API instead
	 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-
	 * command-based.html#defining-commands
	 */
	public class RunAtPower extends Command {
		private Timer m_timer;
		private double m_time;
		private double m_speed;

		/** Creates a new RunTurretAtSpeed. */
		public RunAtPower(double speed, double time) {
			addRequirements(m_turretSubsystem);
			m_timer = new Timer();
			m_time = time;
			m_speed = speed;
		}

		// Called when the command is initially scheduled.
		@Override
		public void initialize() {
			m_timer.reset();
			m_timer.start();
		}

		// Called every time the scheduler runs while the command is scheduled.
		@Override
		public void execute() {
			m_turretSubsystem.runMotorAtDutyCycle(m_speed);
		}

		// Called once the command ends or is interrupted.
		@Override
		public void end(boolean interrupted) {
			m_timer.stop();
			m_turretSubsystem.runMotorAtDutyCycle(0);
		}

		// Returns true when the command should end.
		@Override
		public boolean isFinished() {
			return m_timer.get() >= m_time;
		}
	}

	public class RunAtPowerSignal extends Command {
		private DoubleSupplier m_speed;

		/** Creates a new RunTurretAtSpeed. */
		public RunAtPowerSignal(DoubleSupplier speed) {
			addRequirements(m_turretSubsystem);
			m_speed = speed;
		}

		// Called every time the scheduler runs while the command is scheduled.
		@Override
		public void execute() {
			double speed = m_speed.getAsDouble();
			if (Math.abs(speed) > 0.01) {
				m_turretSubsystem.runMotorAtDutyCycle(speed);
			}
		}

		// Called once the command ends or is interrupted.
		@Override
		public void end(boolean interrupted) {
			m_turretSubsystem.runMotorAtDutyCycle(0);
		}

		// Returns true when the command should end.
		@Override
		public boolean isFinished() {
			return false;
		}
	}
}
