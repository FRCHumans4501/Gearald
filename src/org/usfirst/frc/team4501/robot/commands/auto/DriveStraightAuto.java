package org.usfirst.frc.team4501.robot.commands.auto;

import org.usfirst.frc.team4501.robot.Robot;
import org.usfirst.frc.team4501.robot.commands.arm.PositionArm4Score;
import org.usfirst.frc.team4501.robot.commands.drivetrain.ShiftGearsHigh;
import org.usfirst.frc.team4501.robot.commands.drivetrain.ShiftGearsLow;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class DriveStraightAuto extends CommandGroup {

    public DriveStraightAuto() {
    	requires(Robot.driveTrain);
    	addSequential(new ShiftGearsHigh());
    	addSequential(new PositionArm4Score());
    	addSequential(new DriveStraight4Time(1.5, .8));
    }
}
