// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PhysicalSwerveSim;

public class Robot extends TimedRobot {
	private Command m_autonomousCommand;
	private CommandScheduler m_scheduler = CommandScheduler.getInstance();

	private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
	private final SendableChooser<Command> m_autoChooser = new SendableChooser<Command>();
	private final CommandPS5Controller m_joystick = new CommandPS5Controller(
			Constants.ControllerConstants.kDriverControllerPort);
	private final PhysicalSwerveSim m_swerveSim = new PhysicalSwerveSim();
	private final Timer m_timer = new Timer();

	public Robot() {
		BindDriveControls();
	}

	private void BindDriveControls() {
		m_driveSubsystem.setDefaultCommand(
				m_driveSubsystem.driveCommand(
						() -> -m_joystick.getLeftY(), () -> -m_joystick.getLeftX(),
						() -> m_joystick.getL2Axis() - m_joystick.getR2Axis(), m_joystick.getHID()::getCreateButton));
		m_joystick.circle().onTrue(m_driveSubsystem.resetHeading());
	}

	@Override
	public void robotPeriodic() {
		SwerveModuleState state = m_swerveSim.getModuleState();
		SmartDashboard.putNumber("Velocity (m per s)", state.speedMetersPerSecond);
		SmartDashboard.putNumber("Angle (deg)", state.angle.getDegrees());
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
		m_autonomousCommand = m_autoChooser.getSelected();

		if (m_autonomousCommand != null) {
			m_scheduler.schedule(m_autonomousCommand);
		}
	}

	@Override
	public void autonomousPeriodic() {

	}

	@Override
	public void autonomousExit() {
	}

	@Override
	public void teleopInit() {
		m_timer.start();
		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}
	}

	@Override
	public void teleopPeriodic() {
		double volts = m_timer.get() / 5;
		SmartDashboard.putNumber("Voltage", volts);
		m_swerveSim.simulate(getPeriod(), 0, volts);
	}

	@Override
	public void teleopExit() {
	}

	@Override
	public void testInit() {
		CommandScheduler.getInstance().cancelAll();
		CommandScheduler.getInstance()
				.schedule(ABBA.testBrownoutPreventionCommand(), ChineseRTCalculator.testCommand());
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
