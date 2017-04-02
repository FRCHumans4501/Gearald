package org.usfirst.frc.team4501.robot.subsystems;

import org.usfirst.frc.team4501.robot.RobotMap;
import org.usfirst.frc.team4501.robot.commands.shooter.ShootStop;
import org.usfirst.frc.team4501.robot.subsystems.Shooter.ShooterMotor;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class Shooter extends Subsystem {
	// PID constants
	double kTargetEncoderRate = -75000;
	double kMinShooterEncoderRate = kTargetEncoderRate * 0.95;
	double kMaxShooterEncoderRate = kTargetEncoderRate * 1.00;
	double lastMotorOutput;
	double shooterEncoderRate;
	
	// PID Object
	ShooterMotor shooterMotor;

	// Talon Objects
	Talon shootTalon, intakeTalon;

	// Encoder Object
	Encoder shooterEncoder;
	
	
	// ---------------------------------------------------------------
	// S P I N - U P  P I D
	// ---------------------------------------------------------------
	class ShooterMotor extends PIDSubsystem {
		public ShooterMotor(double p, double i, double d) {
			super("ShooterMotor", p, i, d);
			getPIDController().setContinuous(false);
			getPIDController().setOutputRange(-1, -0.5);
			setSetpoint(kTargetEncoderRate);
			LiveWindow.addActuator("ShooterMotor", "pid", getPIDController());
		}

		@Override
		public void enable() {
			super.enable();
			resetEncoder();
		}

		@Override
		public void disable() {
			super.disable();
			shootTalon.set(0);
		}

		@Override
		protected double returnPIDInput() {
			return readEncoder();
		}

		@Override
		protected void usePIDOutput(double output) {
			lastMotorOutput = -output;
			shootTalon.set(-output);
		}

		@Override
		protected void initDefaultCommand() {
		}
	}
	
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	public Shooter() {
		// Initialization of Talons.
		shootTalon = new Talon(RobotMap.Motors.FLYWEEL);
		intakeTalon = new Talon(RobotMap.Motors.INTAKE);

		// Initialization of the Encoder that the PID uses.
		shooterEncoder = new Encoder(RobotMap.Sensors.SHOOTER_ENCODER_A, RobotMap.Sensors.SHOOTER_ENCODER_B);
	}

	public void enable() {
		if (shooterMotor == null) {
			shooterMotor = new ShooterMotor(1, 0, 0);
		}
		
		shooterMotor.enable();
	}
	
	public void disable() {
		shooterMotor.disable();
		intakeTalon.set(0);
	}

	
	public void resetEncoder() {
		shooterEncoder.reset();
	}
	
	public double readEncoder() {
		shooterEncoderRate = shooterEncoder.getRate();
		SmartDashboard.putNumber("Encoder Rate", shooterEncoderRate);
		//System.out.println("Last Motor Output: " + lastMotorOutput + " EncoderRate: " + shooterEncoderRate);
		return shooterEncoderRate;
	}

	public void shoot() {
		if (shooterEncoderRate <= kMinShooterEncoderRate) {
			intakeTalon.set(-1);
		} else {
			intakeTalon.set(0);
		}
	}
	
	public void shootStop() {
		shootTalon.set(0);
		//disable();
	}	
	
	public void intake(double speed) {
		intakeTalon.set(speed);
	}
	
	public void intakeStop() {
		intakeTalon.set(0);
	}

    public void initDefaultCommand() {
    	setDefaultCommand(new ShootStop());
    }
}

