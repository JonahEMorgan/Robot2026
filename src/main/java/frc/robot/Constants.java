package frc.robot;

import static frc.robot.Robot.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class Constants {
	public static final boolean kLogging = true;

	public static final class Subsystems {
		public static final class TurretConstants {
			public static final int kMotor = 21;
			public static final double kMaxDutyCycle = 0.5;
			public static final double kMinPower = 0.025;
			public static final double kMaxPower = 0.25;
			public static final double kMaxErr = 25;
			public static final double kTolerance = 3;
			public static final double kP = 0.006;
			public static final double kI = 0.0000;
			public static final double kLargeDeadzone = 0.5; // For the X/Y joystick control
			public static final double kSmallDeadzone = 0.05;
			public static final int kSmartCurrent = 20;
			public static final int kCurrent = 25;
			public static final double kMinAngle = 50;
			public static final double kMaxAngle = 270;
		}

		public static final class HoodConstants {
			public static final int kMotor = 1;
			public static final double kMaxDutyCycle = 0.5;
			public static final double kMinPower = 0.025;
			public static final double kMaxPower = 0.25;
			public static final double kMaxErr = 25;
			public static final double kTolerance = 1;
			public static final double kP = 0.01;
			public static final double kI = 0.0000;
			public static final double kDeadzone = 0.05;
			public static final int kSmartCurrent = 20;
			public static final int kCurrent = 25;
			public static final double kMinAngle = 0;
			public static final double kMaxAngle = 38;
		}

		public static final class ShooterConstants {
			public static final int kMotorPort = 51;
			public static final double kCurrentLimit = 30;
			public static final double kV = 480;
			public static final int kDefaultRPM = 2400; // TODO: Test and find actual default RPM
			public static final double kRampRate = 500;
		}

		public static final class IntakeConstants {
			public static final int kIntakeWheelsPort = 56; // TODO: Update CAN IDs, these are placeholder values
			public static final int kIntakeArmPort = 1;
			public static final double kArmConversionFactor = 0.01;

			public static final int kWheelSmartCurrentLimit = 10;
			public static final int kWheelSecondaryCurrentLimit = 20;
			public static final boolean kWheelInvert = false;

			public static final int kArmSmartCurrentLimit = 10;
			public static final int kArmSecondaryCurrentLimit = 20;
			public static final boolean kArmInvert = false;

			public static final double kArmPower = 0.5;
			public static final double kArmRetractRotations = 0;
			public static final double kArmDeployRotations = 3;
		}

		public static final class TransportConstants {
			public static final int kMotorPort = 21;
			public static final int kCurrentLimit = 15;
		}
	}

	public static final class ControllerConstants {
		public static final int kDriverControllerPort = 0;
		public static final int kOperatorControllerPort = 1;
	}

	public static final class DriveConstants {
		public static final double kDeadzone = 0.05;

		// CAN IDs (updated)
		public static final class FrontRight {
			public static final int kDrivePort = 10;
			public static final int kSteerPort = 11;
			public static final int kCANCoderPort = 12;
			public static final boolean kInverted = false;
		}

		public static final class FrontLeft {
			public static final int kDrivePort = 40;
			public static final int kSteerPort = 41;
			public static final int kCANCoderPort = 42;
			public static final boolean kInverted = true;
		}

		public static final class BackRight {
			public static final int kDrivePort = 20;
			public static final int kSteerPort = 21;
			public static final int kCANCoderPort = 22;
			public static final boolean kInverted = false;
		}

		public static final class BackLeft {
			public static final int kDrivePort = 30;
			public static final int kSteerPort = 31;
			public static final int kCANCoderPort = 32;
			public static final boolean kInverted = true;
		}

		// TODO: Make sure these are tuned (can do with SysId)
		public static final double kP = 0.01;
		public static final double kI = 0;
		public static final double kD = 0;
		public static final double kV = 0.12;
		public static final double kA = 0.009;

		public static final double kDriveGearRatio = 6.03;
		public static final double kSteerGearRatio = 26;
		public static final double kWheelDiameter = Units.inchesToMeters(4);
		public static final double kWheelCircumference = Math.PI * kWheelDiameter;

		public static final double kMetersPerMotorRotation = kWheelCircumference / kDriveGearRatio;

		public static final double kMaxThrottle = (isCompBot) ? 1 : 0.5; // Adjust max throttle if needed

		// https://docs.wpilib.org/en/latest/docs/software/basic-programming/coordinate-system.html
		public static final double kModuleDistFromCenter = Units.inchesToMeters(14.5); // Width/2
		public static final Translation2d kFrontLeftLocation = new Translation2d(kModuleDistFromCenter,
				kModuleDistFromCenter);
		public static final Translation2d kFrontRightLocation = new Translation2d(kModuleDistFromCenter,
				-kModuleDistFromCenter);
		public static final Translation2d kBackLeftLocation = new Translation2d(-kModuleDistFromCenter,
				kModuleDistFromCenter);
		public static final Translation2d kBackRightLocation = new Translation2d(-kModuleDistFromCenter,
				-kModuleDistFromCenter);

		public static final TalonFXConfiguration kDriveConfig = new TalonFXConfiguration();
		static {
			kDriveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
			kDriveConfig.CurrentLimits.StatorCurrentLimit = 70; // Higher to prevent torque loss at low speeds
			kDriveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
			kDriveConfig.CurrentLimits.SupplyCurrentLimit = 60; // Lower to prevent brownouts at high speeds
			kDriveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
			kDriveConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		}

		public static final TalonFXConfiguration kSteerConfig = new TalonFXConfiguration();
		static {
			kSteerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
			kSteerConfig.CurrentLimits.StatorCurrentLimit = 50; // Higher to prevent torque loss at low speeds
			kSteerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
			kSteerConfig.CurrentLimits.SupplyCurrentLimit = 40; // Lower to prevent brownouts at high speeds
			kSteerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
			kSteerConfig.MotorOutput.Inverted = (isCompBot) ? InvertedValue.CounterClockwise_Positive
					: InvertedValue.Clockwise_Positive;
		}
	}
}
