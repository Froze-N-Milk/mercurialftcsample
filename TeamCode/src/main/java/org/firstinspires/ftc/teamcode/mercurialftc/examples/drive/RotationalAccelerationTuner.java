package org.firstinspires.ftc.teamcode.mercurialftc.examples.drive;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.mercurialftc.mercurialftc.scheduler.OpModeEX;
import org.mercurialftc.mercurialftc.scheduler.commands.LambdaCommand;
import org.mercurialftc.mercurialftc.scheduler.triggers.Trigger;
import org.mercurialftc.mercurialftc.scheduler.triggers.gamepadex.ContinuousInput;
import org.mercurialftc.mercurialftc.silversurfer.geometry.Pose2D;

@TeleOp(name = "Rotational Acceleration Tuner")
public class RotationalAccelerationTuner extends OpModeEX {
	private MecanumDriveBase mecanumDriveBase;
	
	private double previousTime;
	private double velocity, endVelocity;
	private int velocityIndex, velocitiesSize;
	private double[] velocities;
	private double startTime, endTime;
	private double turn;
	
	/**
	 * called before {@link #initEX()}, solely for initialising all subsystems, ensures that they are registered with the correct {@link org.mercurialftc.mercurialftc.scheduler.Scheduler}, and that their init methods will be run
	 */
	@Override
	public void registerSubsystems() {
		mecanumDriveBase = new MecanumDriveBase(
				this,
				new Pose2D(),
				new ContinuousInput(() -> 0),
				new ContinuousInput(() -> 0),
				new ContinuousInput(() -> turn)
		);
	}
	
	/**
	 * should contain your regular init code
	 */
	@Override
	public void initEX() {
		velocitiesSize = 15;
		velocities = new double[velocitiesSize];
		velocityIndex = 0;
		turn = 1;
	}
	
	/**
	 * registers triggers after the subsystem and regular init code,
	 * useful for organisation of your OpModeEX, but functionally no different to initialising them at the end of {@link #initEX()}
	 */
	@Override
	public void registerTriggers() {
		new Trigger(() -> Math.abs(velocity - mecanumDriveBase.getMotionConstants().getMaxRotationalVelocity()) < 0.01 * mecanumDriveBase.getMotionConstants().getMaxRotationalVelocity())
				.setCommand(
						new LambdaCommand()
								.init(() -> {
									endTime = getElapsedTime().seconds();
									endVelocity = velocity;
									turn = 0;
								})
								.execute(() -> telemetry.addData("acceleration", endVelocity / (endTime - startTime)))
								.finish(() -> false)
				);
	}
	
	@Override
	public void init_loopEX() {
	
	}
	
	@Override
	public void startEX() {
		telemetry.setAutoClear(true);
		startTime = getElapsedTime().seconds();
	}
	
	@Override
	public void loopEX() {
		double currentTime = getElapsedTime().seconds();
		telemetry.addData("max velocity", mecanumDriveBase.getMotionConstants().getMaxRotationalVelocity());
		velocities[velocityIndex] = Math.abs(mecanumDriveBase.getTracker().getPose2D().getTheta().getRadians() - mecanumDriveBase.getTracker().getPreviousPose2D().getTheta().getRadians()) / (currentTime - previousTime);
		velocityIndex++;
		velocityIndex %= velocitiesSize - 1;
		velocity = 0;
		for (double v : velocities) {
			velocity += v;
		}
		velocity /= velocitiesSize;
		telemetry.addData("velocity", velocity);
		telemetry.addData("velocity delta", velocity - mecanumDriveBase.getMotionConstants().getMaxRotationalVelocity());
		previousTime = currentTime;
	}
	
	@Override
	public void stopEX() {
	
	}
}
