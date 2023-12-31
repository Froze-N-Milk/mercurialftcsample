package org.firstinspires.ftc.teamcode.mercurialftc.examples;

import androidx.annotation.NonNull;

import org.mercurialftc.mercurialftc.scheduler.commands.Command;
import org.mercurialftc.mercurialftc.scheduler.commands.LambdaCommand;
import org.mercurialftc.mercurialftc.util.hardware.cachinghardwaredevice.CachingDcMotorEX;
import org.mercurialftc.mercurialftc.scheduler.OpModeEX;
import org.mercurialftc.mercurialftc.scheduler.subsystems.Subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.function.DoubleSupplier;

public class DemoSubsystem extends Subsystem {
	private DcMotorEx motor1, motor2;
	private int position, targetPosition, previousError;
	private double previousTime;
	private double i;
	
	public DemoSubsystem(OpModeEX opModeEX) {
		super(opModeEX);
	}
	
	public int getPosition() {
		return position;
	}
	
	public int getTargetPosition() {
		return targetPosition;
	}
	
	@Override
	public void init() {
		// theses are caching motor wrappers, they ensure that you don't perform redundant hardware writes
		motor1 = new CachingDcMotorEX(opModeEX.hardwareMap.get(DcMotorEx.class, "motor1"));
		motor2 = new CachingDcMotorEX(opModeEX.hardwareMap.get(DcMotorEx.class, "motor2"));
		
		resetEncodersInternal();
		
		setDefaultCommand(runTo(position));
	}
	
	@Override
	public void periodic() {
		position = motor1.getCurrentPosition();
	}
	
	@Override
	public void defaultCommandExecute() {
		double kp = 1;
		double ki = 1;
		double kd = 1;
		
		int error = targetPosition - position;
		double time = opModeEX.getElapsedTime().seconds();
		
		double p = kp * error;
		i += ki * (error * (time - previousTime));
		double d = kd * ((error - previousError) / (time - previousTime));
		
		double output = p + i + d;
		motor1.setPower(output);
		motor2.setPower(output);
		
		previousTime = time;
		previousError = error;
	}
	
	/**
	 * sets up the PID loop handled in the default command to run to a new target position
	 * <p>note that this command does not require the subsystem, this is because it should not be interrupting as it does not need to manipulate the hardware devices directly, and there is no harm in running it, it also ends instantly</p>
	 *
	 * @param targetPosition new target position
	 * @return a command with the specified behaviour
	 */
	public Command runTo(int targetPosition) {
		return new LambdaCommand()
				.setInterruptible(true)
				.setInit(() -> prepPID(targetPosition))
				.setExecute(this::defaultCommandExecute) // uses the default command execute
				.setFinish(() -> false);
	}
	
	public Command runTo(@NonNull Positions position) {
		return runTo(position.getPosition());
	}
	
	public Command manualControl(DoubleSupplier controller) {
		return new LambdaCommand()
				.setRequirements(this)
				.setRunStates(OpModeEX.OpModeEXRunStates.LOOP)
				.setInterruptible(true)
				.setExecute(() -> {
					motor1.setPower(controller.getAsDouble());
					motor2.setPower(controller.getAsDouble());
				})
				.setFinish(() -> false);
	}
	
	public Command resetEncoders() {
		return new LambdaCommand()
				.setInterruptible(false)
				.setRunStates(OpModeEX.OpModeEXRunStates.INIT_LOOP, OpModeEX.OpModeEXRunStates.LOOP)
				.setInit(this::resetEncodersInternal)
				.setFinish(() -> true);
	}
	
	private void resetEncodersInternal() {
		motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
	}
	
	public void prepPID(int targetPosition) {
		this.targetPosition = targetPosition;
		this.i = 0;
		this.previousTime = opModeEX.getElapsedTime().seconds();
		this.previousError = targetPosition - position;
	}
	
	@Override
	public void close() {
	
	}
	
	public enum Positions {
		ZERO(0),
		POS1(100),
		POS2(200),
		POS3(300);
		private final int position;
		
		Positions(int position) {
			this.position = position;
		}
		
		public int getPosition() {
			return position;
		}
	}
}
