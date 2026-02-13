// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.RunTurretToAngleHardware;
import frc.robot.commands.ShooterCommand;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class Robot extends TimedRobot {
	private CommandScheduler m_scheduler = CommandScheduler.getInstance();

	private final Shooter m_shooterSubsystem = new Shooter();
	private final Turret m_turretSubsystem = new Turret();
	private final CommandPS5Controller m_driverController = new CommandPS5Controller(
			Constants.ControllerConstants.kDriverControllerPort);
	private final CommandPS5Controller m_operatorController = new CommandPS5Controller(
			Constants.ControllerConstants.kOperatorControllerPort);

	{
		new Drive();
	}

	public Robot() {
		bindControls();
	}

	private void bindControls() {
		Drive.getDrive().setDefaultCommand(
				new DriveCommands.JoystickDrive(
						() -> -m_driverController.getLeftY(), () -> -m_driverController.getLeftX(),
						() -> m_driverController.getL2Axis() - m_driverController.getR2Axis(),
						m_driverController.getHID()::getCreateButton));
	}

	@Override
	public void robotPeriodic() {
		m_scheduler.run();

		SmartDashboard.putData(m_scheduler);
	}

	@Override
	public void disabledInit() {
	}

	@Override
	public void disabledPeriodic() {

	}

	@Override
	public void disabledExit() {

	}

	@Override
	public void autonomousInit() {
		m_scheduler.cancelAll();
		m_scheduler.schedule(
				Commands.parallel(
						Commands.sequence(
								new RunTurretToAngleHardware(m_turretSubsystem, 45),
								Commands.waitSeconds(1),
								new RunTurretToAngleHardware(m_turretSubsystem, 225),
								Commands.waitSeconds(1),
								new RunTurretToAngleHardware(m_turretSubsystem, 45),
								Commands.waitSeconds(1),
								new RunTurretToAngleHardware(m_turretSubsystem, 225)),
						new ShooterCommand.RunAtDynamicRPM(m_shooterSubsystem, 2400).withTimeout(40)));
	}

	@Override
	public void teleopInit() {
		m_scheduler.cancelAll();
	}

	@Override
	public void testInit() {
		m_scheduler.cancelAll();
		m_scheduler.schedule(Commands.sequence(ClampedP.testCommand(), ABBA.testBrownoutPreventionCommand()));
	}
}
