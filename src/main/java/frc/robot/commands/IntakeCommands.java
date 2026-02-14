package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Subsystems.IntakeConstants;
import frc.robot.subsystems.Intake;

public class IntakeCommands {
	public static class ExtendArmCommand extends Command {
		public ExtendArmCommand() {
			setName("Extend Intake Arm");
			addRequirements(Intake.getIntake());
		}

		public void initialize() {
			Intake.setArmPower(IntakeConstants.kArmPower);
		}

		// Update this to use limit switch instead of getArmAngle method.
		public boolean isFinished() {
			return Intake.isForwardLimitActive();
		}

		public void end() {
			Intake.stopArm();
		}
	}

	public static class RetractArmCommand extends Command {
		public RetractArmCommand() {
			setName("Retract Intake Arm");
			addRequirements(Intake.getIntake());
		}

		public void initialize() {
			Intake.setArmPower(-IntakeConstants.kArmPower);
		}

		// Update this to use limit switch instead of getArmAngle method.
		public boolean isFinished() {
			return Intake.isReverseLimitActive();
		}

		public void end() {
			Intake.stopArm();
		}
	}

	public static class Spin extends Command {
		private final double m_speed;

		public Spin(double speed) {
			m_speed = speed;
			setName("Spin Intake");
		}

		public void initialize() {
			Intake.setWheelPower(m_speed);
		}

		// Update this to use limit switch instead of getArmAngle method.
		public boolean isFinished() {
			return false;
		}

		public void end() {
			Intake.stopWheel();
		}
	}
}