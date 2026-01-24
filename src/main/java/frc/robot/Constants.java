package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class Constants {
	public static final class ControllerConstants {
		public static final int kDriverControllerPort = 0;
		public static final int kOperatorControllerPort = 1;
		public static final double kDeadzone = 0.05;
		public static final double kTriggerDeadzone = 0.05;
	}

	public static final class DriveConstants {
		// CAN IDs (updated)
		public static final int kFrontRightDrivePort = 10;
		public static final int kFrontRightSteerPort = 11;
		public static final int kFrontLeftDrivePort = 40;
		public static final int kFrontLeftSteerPort = 41;
		public static final int kBackRightDrivePort = 20;
		public static final int kBackRightSteerPort = 21;
		public static final int kBackLeftDrivePort = 30;
		public static final int kBackLeftSteerPort = 31;
		public static final int kFrontRightCANCoderPort = 12;
		public static final int kFrontLeftCANCoderPort = 42;
		public static final int kBackRightCANCoderPort = 22;
		public static final int kBackLeftCANCoderPort = 32;

		// TODO: Make sure these are tuned (can do with SysId)
		public static final double kP = 0.09;
		public static final double kI = 0.0;
		public static final double kD = 0.001;
		public static final double kS = 0;
		public static final double kV = 0.12;
		public static final double kA = 0.009;

		public static final double kRotationP = 5; // TODO: tune it
		public static final double kRotationI = 0.0;
		public static final double kRotationD = 0.1; // TODO: tune it
		public static final double kRotationS = 0;
		public static final double kRotationV = 1.9;
		public static final double kRotationA = 0.009;

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

		public static final double kRampRate = .1;
		private static final TalonFXConfiguration kBaseConfig = new TalonFXConfiguration();
		static {
			kBaseConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
			kBaseConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
			kBaseConfig.CurrentLimits.StatorCurrentLimitEnable = true;
			kBaseConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = kRampRate;
			kBaseConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = kRampRate;
		}
		public static final TalonFXConfiguration kDriveConfig = kBaseConfig.clone();
		static {
			kDriveConfig.CurrentLimits.SupplyCurrentLimit = 45; // For avoiding brownout
			kDriveConfig.CurrentLimits.SupplyCurrentLowerLimit = 45;
			kDriveConfig.CurrentLimits.StatorCurrentLimit = 80; // Output current (proportional to acceleration)
		}
		public static final TalonFXConfiguration kSteerConfig = kBaseConfig.clone();
		static {
			kSteerConfig.CurrentLimits.StatorCurrentLimit = 60;
			kSteerConfig.CurrentLimits.SupplyCurrentLimit = 75;
		}
		public static final double kTeleopDriveMaxSpeed = 12.0; // 5 meters per second
		public static final double kTeleopTurnMaxAngularSpeed = Math.toRadians(360 * 5);
	}
}
