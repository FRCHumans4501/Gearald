package org.usfirst.frc.team4501.robot.commands.arm;

import org.usfirst.frc.team4501.robot.OI;
import org.usfirst.frc.team4501.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Gives the operator some control over the arms movement.
 */
public class ManualArmMove extends Command {

	OI oi;
	
    public ManualArmMove() {
		requires(Robot.arm);
		oi = Robot.oi;

	}

	// Called just before this Command runs the first time
	protected void initialize() {
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		double y = (Math.sin(.8 * oi.getShooterY()));
		Robot.arm.moveArm(y);
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
