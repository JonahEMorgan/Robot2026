package frc.robot.commands;

import static frc.robot.Constants.Subsystems.ClimberConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class ClimberCommands extends PositionControlCommands<Climber> {
	public static class RunAtPower extends Command {
		private double m_power;

		public RunAtPower(double power) {
			setName("Run Climber At Power");
			m_power = power;
			addRequirements(Climber.getClimber());
		}

		@Override
		public void initialize() {
			Climber.getClimber().setMotorPower(m_power);
		}

		@Override
		public void end(boolean interrupted) {
			Climber.getClimber().stopMotor();
		}

		@Override
		public boolean isFinished() {
			return false;
		}
	}

	public static class RunToHeightSoftware extends Command {
		private final double m_revolutions;
		private final Climber m_climber;
		private final static double tolerance = 0.1;
		private final static double speed = 0.1;

		public RunToHeightSoftware(double height) {
			setName("Run Climber to Height");
			m_revolutions = height * kGearRatio;
			m_climber = Climber.getClimber();
			addRequirements(m_climber);
		}

		@Override
		public void initialize() {
			m_climber.setMotorPower(m_climber.getMotorRotations() < m_revolutions ? speed : -speed);
		}

		@Override
		public void end(boolean interrupted) {
			m_climber.stopMotor();
		}

		@Override
		public boolean isFinished() {
			return Math.abs(m_climber.getMotorRotations() - m_revolutions) < tolerance;
		}
	}
}
