package org.usfirst.frc.team4501.robot.commands.drivetrain;

import org.usfirst.frc.team4501.robot.OI;
import org.usfirst.frc.team4501.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Opperator controlls for driving.
 */
public class ArcadeDrive extends Command {
	OI oi;

	public ArcadeDrive() {
		requires(Robot.driveTrain);
		oi = new OI();
	}

	// Called just before this Command runs the first time
	protected void initialize() {
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		Robot.driveTrain.arcadeDrive(oi.getTriggers(), oi.getLeftXboxX());
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
