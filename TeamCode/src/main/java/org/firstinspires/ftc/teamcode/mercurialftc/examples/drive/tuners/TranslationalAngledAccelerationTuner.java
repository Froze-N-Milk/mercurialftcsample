package org.firstinspires.ftc.teamcode.mercurialftc.examples.drive.tuners;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mercurialftc.examples.drive.MecanumDriveBase;
import org.mercurialftc.mercurialftc.scheduler.OpModeEX;
import org.mercurialftc.mercurialftc.scheduler.commands.LambdaCommand;
import org.mercurialftc.mercurialftc.scheduler.triggers.Trigger;
import org.mercurialftc.mercurialftc.scheduler.triggers.gamepadex.ContinuousInput;
import org.mercurialftc.mercurialftc.silversurfer.geometry.angle.AngleDegrees;
import org.mercurialftc.mercurialftc.silversurfer.geometry.Pose2D;
import org.mercurialftc.mercurialftc.silversurfer.geometry.Vector2D;
import org.mercurialftc.mercurialftc.silversurfer.tracker.Tracker;

@TeleOp(name = "Translational Angled Acceleration Tuner")
public class TranslationalAngledAccelerationTuner extends OpModeEX {
	private MecanumDriveBase mecanumDriveBase;
	
	private double previousTime;
	private double velocity, endVelocity;
	private int velocityIndex, velocitiesSize;
	private double[] velocities;
	private double startTime, endTime;
	private double drive;
	
	/**
	 * called before {@link #initEX()}, solely for initialising all subsystems, ensures that they are registered with the correct {@link org.mercurialftc.mercurialftc.scheduler.Scheduler}, and that their init methods will be run
	 */
	@Override
	public void registerSubsystems() {
		mecanumDriveBase = new MecanumDriveBase(
				this,
				new Pose2D(0, 0, new AngleDegrees(45)),
				new ContinuousInput(() -> 0),
				new ContinuousInput(() -> drive),
				new ContinuousInput(() -> 0)
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
		drive = 1;
	}
	
	/**
	 * registers triggers after the subsystem and regular init code,
	 * useful for organisation of your OpModeEX, but functionally no different to initialising them at the end of {@link #initEX()}
	 */
	@Override
	public void registerTriggers() {
		new Trigger(() -> Math.abs(velocity - mecanumDriveBase.getMotionConstants().getMaxTranslationalAngledVelocity()) < 0.02 * mecanumDriveBase.getMotionConstants().getMaxTranslationalAngledVelocity())
				.setCommand(
						new LambdaCommand()
								.init(() -> {
									endTime = getElapsedTime().seconds();
									endVelocity = velocity;
									drive = 0;
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
		mecanumDriveBase.getTracker().reset();
		telemetry.setAutoClear(true);
		startTime = getElapsedTime().seconds();
	}
	
	@Override
	public void loopEX() {
		double currentTime = getElapsedTime().seconds();
		telemetry.addData("max velocity", mecanumDriveBase.getMotionConstants().getMaxTranslationalAngledVelocity());
		Tracker tracker = mecanumDriveBase.getTracker();
		Vector2D velocityVector = new Vector2D(tracker.getPose2D().getX() - tracker.getPreviousPose2D().getX(), tracker.getPose2D().getY() - tracker.getPreviousPose2D().getY());
		double translationalVelocity = velocityVector.getMagnitude() / (currentTime - previousTime);
		velocities[velocityIndex] = translationalVelocity;
		velocityIndex++;
		velocityIndex %= velocitiesSize;
		velocity = 0;
		for (double v : velocities) {
			velocity += v;
		}
		velocity /= velocitiesSize;
		telemetry.addData("velocity", velocity);
		telemetry.addData("velocity delta", velocity - mecanumDriveBase.getMotionConstants().getMaxTranslationalAngledVelocity());
		previousTime = currentTime;
	}
	
	@Override
	public void stopEX() {
	
	}
}
