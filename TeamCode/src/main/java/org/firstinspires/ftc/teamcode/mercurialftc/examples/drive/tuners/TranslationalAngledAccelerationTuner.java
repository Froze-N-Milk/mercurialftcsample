package org.firstinspires.ftc.teamcode.mercurialftc.examples.drive.tuners;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mercurialftc.examples.drive.MecanumDriveBase;
import org.mercurialftc.mercurialftc.scheduler.OpModeEX;
import org.mercurialftc.mercurialftc.scheduler.commands.LambdaCommand;
import org.mercurialftc.mercurialftc.scheduler.bindings.Trigger;
import org.mercurialftc.mercurialftc.scheduler.bindings.gamepadex.DomainSupplier;
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
	
	@Override
	public void registerSubsystems() {
		mecanumDriveBase = new MecanumDriveBase(
				this,
				new Pose2D(0, 0, new AngleDegrees(45)),
				new DomainSupplier(() -> 0),
				new DomainSupplier(() -> drive),
				new DomainSupplier(() -> 0)
		);
	}
	
	@Override
	public void initEX() {
		velocitiesSize = 15;
		velocities = new double[velocitiesSize];
		velocityIndex = 0;
		drive = 1;
	}
	
	@Override
	public void registerBindings() {
		new Trigger(() -> Math.abs(velocity - mecanumDriveBase.getMotionConstants().getMaxTranslationalAngledVelocity()) < 0.02 * mecanumDriveBase.getMotionConstants().getMaxTranslationalAngledVelocity())
				.setCommand(
						new LambdaCommand()
								.setInit(() -> {
									endTime = getElapsedTime().seconds();
									endVelocity = velocity;
									drive = 0;
								})
								.setExecute(() -> telemetry.addData("acceleration", endVelocity / (endTime - startTime)))
								.setFinish(() -> false)
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
		Vector2D velocityVector = tracker.getDeltaPositionVector();
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
