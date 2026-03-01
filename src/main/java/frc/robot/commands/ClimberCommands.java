package frc.robot.commands;

import static frc.robot.Constants.Subsystems.ClimberConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class ClimberCommands {
	/*
	 * You should consider using the more terse Command factories API instead
	 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-
	 * command-based.html#defining-commands
	 */

	public static class RunAtPower extends Command {

		private double m_power;

		/** Creates a new runShooter. */
		public RunAtPower(double power) {
			setName("Run Climber At Power");
			m_power = power;
			addRequirements(Climber.getClimber());
		}

		// Called every time the scheduler runs while the command is scheduled.
		@Override
		public void execute() {
			Climber.getClimber().setMotorPower(m_power);
		}

		// Called once the command ends or is interrupted.
		@Override
		public void end(boolean interrupted) {
			Climber.getClimber().stopMotor();
		}

		// Returns true when the command should end.
		@Override
		public boolean isFinished() {
			return false;
		}
	}

	public static class RunToHeightSoftware extends Command {
		private double m_height;
		private double m_revolutions;

		public RunToHeightSoftware(double height) {
			setName("Run Climber to Height");
			m_height = height;
			m_revolutions = m_height * kGearRatio;
			addRequirements(Climber.getClimber());
		}

		public void execute() {
		}

		public void end(boolean interrupted) {
		}

		public boolean isFinished() {
			return false;
		}
	}
}
