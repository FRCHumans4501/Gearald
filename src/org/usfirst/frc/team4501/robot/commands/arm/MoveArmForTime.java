package org.usfirst.frc.team4501.robot.commands.arm;

import org.usfirst.frc.team4501.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Runs the arm at a specified speed and for a specified amount of time.
 */
public class MoveArmForTime extends Command {

	private double moveSpeed;
	private double runTime;
	
    public MoveArmForTime(double speed, double length) {
    	requires(Robot.arm);
    	
        this.moveSpeed = speed;
        this.runTime = length;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.arm.moveArm(moveSpeed);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return (this.timeSinceInitialized() >= runTime);
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.arm.moveArm(0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
