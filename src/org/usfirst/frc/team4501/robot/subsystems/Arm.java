package org.usfirst.frc.team4501.robot.subsystems;

import org.usfirst.frc.team4501.robot.RobotMap;
import org.usfirst.frc.team4501.robot.commands.arm.ManualArmMove;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Arm extends Subsystem {

	DoubleSolenoid armSolenoid;
	Talon armMotor;
	DigitalInput bottomSwitch, topSwitch;

	public Arm() {
		// Initializes the arm talon.
		armMotor = new Talon(RobotMap.Motors.ARM);

		// Initializes the arm solenoids that control the arms open and close
		// function.
		armSolenoid = new DoubleSolenoid(RobotMap.Solenoids.ARMHIGH, RobotMap.Solenoids.ARMLOW);

		// Initializes the limit switches that prevent the arm from going to
		// far.
		bottomSwitch = new DigitalInput(RobotMap.Sensors.ARM_LOW);
		topSwitch = new DigitalInput(RobotMap.Sensors.ARM_HIGH);
	}

	// A method that opens the arms.
	public void armOpen() {
		armSolenoid.set(DoubleSolenoid.Value.kForward);
	}

	// A method that closes the arms.
	public void armClose() {
		armSolenoid.set(DoubleSolenoid.Value.kReverse);
	}
	
	// A method that checks to see if the arm is in the up position.
	public boolean isArmUp() {
		// For some reason the switch returns true while the circuit is open
		// and false while it's closed. The way we have the limit switches
		// on is that unless its pressed the circuit is open. We don't want
		// that so we get the opposite of the value read.
		return !topSwitch.get();
	}

	// A method that checks to see if the arm is in the down position.
	public boolean isArmDown() {
		// Same thing here as the other switch.
		return !bottomSwitch.get();
	}

	// A method that rotates the arm up and down and limits the arm from going
	// to far in either direction.
	public void moveArm(double speed) {
		speed = -speed;
		if (speed > 0) { 
			if (isArmDown()) {
				armMotor.set(0);
			} else {
				armMotor.set(speed);
			}
		} else if (speed < 0) {
			if (isArmUp()) {
				armMotor.set(0);
			} else {
				armMotor.set(speed);
			}
		} else {
			armMotor.set(0);
		}
	}

	// Set a default command that always is running unless something else
	// requires the subsystem.
	public void initDefaultCommand() {
		setDefaultCommand(new ManualArmMove());
	}
}
