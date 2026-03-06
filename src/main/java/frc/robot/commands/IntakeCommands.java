package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeWheels;

public class IntakeCommands {
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