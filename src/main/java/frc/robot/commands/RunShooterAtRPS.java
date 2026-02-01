// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunShooterAtRPS extends Command {

	private Shooter m_Shooter;
	private double m_rps;

	/** Creates a new runShooter. */
	public RunShooterAtRPS(Shooter shooter, double rps) {
		m_Shooter = shooter;
		m_rps = rps;
		setName("Run Shooter At RPS");
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(shooter);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		m_Shooter.runShooterAtRPS(m_rps);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		m_Shooter.stopShooter();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
