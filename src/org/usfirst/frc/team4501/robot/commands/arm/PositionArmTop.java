package org.usfirst.frc.team4501.robot.commands.arm;

import org.usfirst.frc.team4501.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Auto-magically moves the arm to the top position.
 */
public class PositionArmTop extends Command {
	
	private boolean isDone;
	boolean armUp = false;

    public PositionArmTop() {
		requires(Robot.arm);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		isDone = false;
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		Robot.arm.armClose();
		if (!Robot.arm.isArmUp() && armUp == false) {
			Robot.arm.moveArm(.8);
		} else if (Robot.arm.isArmUp()) {
			armUp = true;
			isDone = true;
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return isDone;
	}

	// Called once after isFinished returns true
	protected void end() {
		armUp = false;
		isDone = false;
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
