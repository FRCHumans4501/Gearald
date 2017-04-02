package org.usfirst.frc.team4501.robot.commands.auto;

import org.usfirst.frc.team4501.robot.Robot;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class NoAuto extends CommandGroup {

    public NoAuto() {
    	requires(Robot.driveTrain);
    	addSequential(new DriveRotate4Time(0, 0));
    }
}
