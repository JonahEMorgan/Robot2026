package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeArm;
import frc.robot.subsystems.IntakeWheels;

public class IntakeCommands extends PositionControlCommands<IntakeArm> {
	public static class SpinIntake extends Command {
		private double m_speed;

		public SpinIntake(double speed) {
			setName("Spin Intake Wheels");
			m_speed = speed;
			addRequirements(IntakeWheels.getIntake());
		}

		@Override
		public void initialize() {
			IntakeWheels.setWheelPower(m_speed);
		}

		@Override
		public void end(boolean interrupted) {
			IntakeWheels.stopWheel();
		}
	}

	public static class StopIntake extends Command {
		public StopIntake() {
			setName("Stop Intake Wheels");
			addRequirements(IntakeWheels.getIntake());
		}

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