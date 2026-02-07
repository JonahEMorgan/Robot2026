// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunTurretToDistance extends Command {
	private Turret m_turretSubsystem;
	private double m_rot;
	private double m_speed;

	/** Creates a new RunTurretAtSpeed. */
	public RunTurretToDistance(Turret turretSubsystem, double speed, double rotations) {
		addRequirements(turretSubsystem);
		m_turretSubsystem = turretSubsystem;
		m_rot = rotations;
		m_speed = speed;
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		SmartDashboard.putNumber("Position", m_turretSubsystem.getPosition());

		m_turretSubsystem.runMotorAtDutyCycle(m_speed);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		m_turretSubsystem.runMotorAtDutyCycle(0);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return m_turretSubsystem.getPosition() / 360 >= m_rot;
	}
}
