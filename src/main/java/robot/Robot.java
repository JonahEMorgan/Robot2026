// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import robot.Constants.ControllerConstants;
import robot.subsystems.DriveSubsystem;
import robot.utilities.CompBot;

public class Robot extends TimedRobot {
	private Command m_autonomousCommand;
	private CommandScheduler m_scheduler = CommandScheduler.getInstance();

	private final DriveSubsystem m_driveSubsystem = new DriveSubsystem(CompBot::new);
	private final SendableChooser<Command> m_autoChooser = new SendableChooser<>();
	private final CommandPS5Controller m_joystick = new CommandPS5Controller(ControllerConstants.kDriverControllerPort);

	public static void main(String... args) {
		RobotBase.startRobot(Robot::new);
	}

	public Robot() {
		BindDriveControls();
	}

	private void BindDriveControls() {
		m_driveSubsystem.setDefaultCommand(
				m_driveSubsystem.driveCommand(
						() -> -m_joystick.getLeftY(), () -> -m_joystick.getLeftX(),
						() -> m_joystick.getL2Axis() - m_joystick.getR2Axis(), m_joystick.getHID()::getCreateButton));
	}

	public void robotPeriodic() {
		m_scheduler.run();
		SmartDashboard.putData(m_scheduler);
	}

	public void autonomousInit() {
		m_autonomousCommand = m_autoChooser.getSelected();
		if (m_autonomousCommand != null)
			m_scheduler.schedule(m_autonomousCommand);
	}

	public void teleopInit() {
		if (m_autonomousCommand != null)
			m_autonomousCommand.cancel();
	}
}
