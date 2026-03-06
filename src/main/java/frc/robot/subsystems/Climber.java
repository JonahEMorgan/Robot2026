package frc.robot.subsystems;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Subsystems.ClimberConstants;
import frc.robot.Constants.Subsystems.IntakeConstants;
import frc.robot.commands.ClimberCommands;

public class Climber extends PositionControlSubsystem {
	private static Climber s_theClimber;
	private static final ClimberCommands s_commands = new ClimberCommands();

	public Climber() {
		super(ClimberConstants.kClimberPort);

		SparkMaxConfig config = new SparkMaxConfig();

		config.smartCurrentLimit(ClimberConstants.kSmartCurrentLimit);
		config.secondaryCurrentLimit(ClimberConstants.kSecondaryCurrentLimit);
		config.idleMode(IdleMode.kBrake);
		config.inverted(ClimberConstants.kInvert);
		config.encoder.positionConversionFactor(ClimberConstants.kGearRatio);
		config.closedLoop.pid(IntakeConstants.kP, 0, 0);
		config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
		config.closedLoop.positionWrappingEnabled(false);

		configureMotor(config);

		if (s_theClimber == null) {
			s_theClimber = this;
		} else {
			throw new Error("Intake Arm subsystem is already constructed.");
		}
	}

	public static Climber getClimber() {
		return s_theClimber;
	}

	public static Command getZeroCommand() { // TODO: Find actual power and time needed
		return new SequentialCommandGroup(
				s_commands.new SpinMotorPowerForTime(s_theClimber, -.3, 5),
				s_commands.new ResetEncoder(s_theClimber));
	}

	public static Command getClimbCommand() {
		return s_commands.new MoveMotorToPosition(s_theClimber, ClimberConstants.kClimbPosition, 0.1, 0.5,
				0.5, 0.125, false);
	}

	public static Command getRetractCommand() {
		return s_commands.new MoveMotorToPosition(s_theClimber, ClimberConstants.kRetractPosition, 0.1,
				0.5, 0.5, 0.125, false);
	}
}