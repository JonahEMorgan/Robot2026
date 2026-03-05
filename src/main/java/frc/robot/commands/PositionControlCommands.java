package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PositionControlSubsystem;

/**
 * We need a bit of a clean up, right now
 */
public class PositionControlCommands {
	public static class MoveMotorToPosition extends Command {
		public MoveMotorToPosition(PositionControlSubsystem subsystem, double a, double b, double c, double d,
				double e, boolean f) {
		}
	}

	public static class SpinMotorPowerForTime extends Command {
		public SpinMotorPowerForTime(PositionControlSubsystem subsystem, double power, double time) {
		}
	}

	public static class ResetEncoder extends Command {
		public ResetEncoder() {
		}
	}
}
