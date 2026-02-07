// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.commands.ShooterCommand;
import frc.robot.subsystems.Shooter;

public class Robot extends TimedRobot {
	private Command m_autonomousCommand;
	private CommandScheduler m_scheduler = CommandScheduler.getInstance();

	/*
	 * private final Drive m_driveSubsystem = new Drive();
	 * private final Transport m_transportSubsystem = new Transport();
	 * private final Intake m_intakeSubsystem = new Intake();
	 */
	private final Shooter m_shooterSubsystem = new Shooter();
	/*
	 * private final Turret m_turretSubsystem = new Turret();
	 * private final Climber m_climberSubsystem = new Climber();
	 */
	private final SendableChooser<Command> m_autoChooser = new SendableChooser<Command>();
	private final CommandPS5Controller m_joystick = new CommandPS5Controller(
			Constants.ControllerConstants.kDriverControllerPort);

	public Robot() {
		BindDriveControls();
	}

	private void BindDriveControls() {
		/*
		 * m_driveSubsystem.setDefaultCommand(
		 * m_driveSubsystem.driveCommand(
		 * () -> -m_joystick.getLeftY(), () -> -m_joystick.getLeftX(),
		 * () -> m_joystick.getL2Axis() - m_joystick.getR2Axis(),
		 * m_joystick.getHID()::getCreateButton));
		 * m_transportSubsystem.setDefaultCommand(
		 * m_transportSubsystem.moveWithTrigger(
		 * m_joystick.triangle(),
		 * m_joystick.cross()));
		 * m_intakeSubsystem.setDefaultCommand(
		 * m_intakeSubsystem.moveWithTrigger(
		 * m_joystick.R1(),
		 * m_joystick.L1()));
		 * m_joystick.circle().onTrue(
		 * m_intakeSubsystem.deployRollers());
		 * m_joystick.circle().onTrue(
		 * m_intakeSubsystem.retractRollers());
		 */
		/*
		 * m_shooterSubsystem.setDefaultCommand(
		 * new ShooterCommand.RunAtPower(m_shooterSubsystem, .1, 10));
		 */
		/*
		 * m_turretSubsystem.setDefaultCommand(
		 * m_turretSubsystem.aimWithJoystick(
		 * m_joystick::getLeftX,
		 * m_joystick::getLeftY));
		 * m_climberSubsystem.setDefaultCommand(
		 * m_climberSubsystem.moveWithTrigger(
		 * m_joystick.povUp(), m_joystick.povDown()));
		 */
	}

	@Override
	public void robotPeriodic() {
		m_scheduler.run();

		SmartDashboard.putData(m_scheduler);
		SmartDashboard.putNumber("rpm", m_shooterSubsystem.getRPM());
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
		/*
		 * m_autonomousCommand = m_autoChooser.getSelected();
		 * if (m_autonomousCommand != null) {
		 * m_scheduler.schedule(m_autonomousCommand);
		 * }
		 */
		CommandScheduler.getInstance().schedule(
				Commands.sequence(
						new ShooterCommand.RunAtDynamicRpm(m_shooterSubsystem, 2400).withTimeout(40)));
	}

	@Override
	public void autonomousPeriodic() {

	}

	@Override
	public void autonomousExit() {
	}

	@Override
	public void teleopInit() {
		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}
	}

	@Override
	public void teleopPeriodic() {

	}

	@Override
	public void teleopExit() {
	}

	@Override
	public void testInit() {
		CommandScheduler.getInstance().cancelAll();
	}

	@Override
	public void testPeriodic() {
	}

	@Override
	public void testExit() {
	}

	@Override
	public void simulationPeriodic() {

	}
}
