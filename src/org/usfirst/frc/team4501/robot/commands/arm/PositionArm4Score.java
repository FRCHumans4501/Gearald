package org.usfirst.frc.team4501.robot.commands.arm;

import org.usfirst.frc.team4501.robot.Robot;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * Puts the arm in a position somewhat ideal to score with.
 */
public class PositionArm4Score extends CommandGroup {

    public PositionArm4Score() {
    	requires(Robot.arm);
    	addSequential(new PositionArmTop());
    	addSequential(new MoveArmForTime(-0.5, 0.2));
    }
}
