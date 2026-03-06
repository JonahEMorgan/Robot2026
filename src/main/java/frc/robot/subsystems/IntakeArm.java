package frc.robot.subsystems;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Subsystems.IntakeConstants;
import frc.robot.commands.IntakeCommands;

public class IntakeArm extends PositionControlSubsystem {
	private static IntakeArm s_theIntakeArm;
	private static final IntakeCommands s_commands = new IntakeCommands();

	public IntakeArm() {
		super(IntakeConstants.kIntakeArmPort);

		SparkMaxConfig config = new SparkMaxConfig();

		config.smartCurrentLimit(IntakeConstants.kArmSmartCurrentLimit);
		config.secondaryCurrentLimit(IntakeConstants.kArmSecondaryCurrentLimit);
		config.idleMode(IdleMode.kBrake);
		config.inverted(IntakeConstants.kArmInvert);
		config.encoder.positionConversionFactor(IntakeConstants.kArmConversionFactor);
		config.closedLoop.pid(IntakeConstants.kP, 0, 0);
		config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
		config.closedLoop.positionWrappingEnabled(false);

		configureMotor(config);

		if (s_theIntakeArm == null) {
			s_theIntakeArm = this;
		} else {
			throw new Error("Intake Arm subsystem is already constructed.");
		}
	}

	public static IntakeArm getIntakeArm() {
		return s_theIntakeArm;
	}

	public static Command getZeroCommand() { // TODO: Find actual power and time needed
		return new SequentialCommandGroup(
				s_commands.new SpinMotorPowerForTime(s_theIntakeArm, -.3, 5),
				s_commands.new ResetEncoder(s_theIntakeArm));
	}

	public static Command getOutCommand() {
		return s_commands.new MoveMotorToPosition(s_theIntakeArm, IntakeConstants.kInPosition, 0.1, 0.7,
				0.5, 0.125, false);
	}

	public static Command getInCommand() {
		return s_commands.new MoveMotorToPosition(s_theIntakeArm, IntakeConstants.kOutPosition, 0.1, 0.7,
				0.5, 0.125, false);
	}
}
