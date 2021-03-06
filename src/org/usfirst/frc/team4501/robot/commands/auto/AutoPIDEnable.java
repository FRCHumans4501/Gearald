package org.usfirst.frc.team4501.robot.commands.auto;

import org.usfirst.frc.team4501.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class AutoPIDEnable extends Command {

    public AutoPIDEnable() {
    	requires(Robot.driveTrain);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.driveTrain.initPIDs();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.driveTrain.updatePIDPeriodic();
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return (Robot.driveTrain.isDone());
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.driveTrain.onEnd();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}