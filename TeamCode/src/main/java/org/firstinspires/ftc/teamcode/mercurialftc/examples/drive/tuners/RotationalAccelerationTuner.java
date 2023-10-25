package org.firstinspires.ftc.teamcode.mercurialftc.examples.drive.tuners;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mercurialftc.examples.drive.MecanumDriveBase;
import org.mercurialftc.mercurialftc.scheduler.OpModeEX;
import org.mercurialftc.mercurialftc.scheduler.commands.LambdaCommand;
import org.mercurialftc.mercurialftc.scheduler.bindings.Trigger;
import org.mercurialftc.mercurialftc.scheduler.bindings.gamepadex.DomainSupplier;
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
	
	@Override
	public void registerSubsystems() {
		mecanumDriveBase = new MecanumDriveBase(
				this,
				new Pose2D(),
				new DomainSupplier(() -> 0),
				new DomainSupplier(() -> 0),
				new DomainSupplier(() -> turn)
		);
	}
	
	@Override
	public void initEX() {
		velocitiesSize = 15;
		velocities = new double[velocitiesSize];
		velocityIndex = 0;
		turn = 1;
	}
	
	@Override
	public void registerBindings() {
		new Trigger(() -> Math.abs(velocity - mecanumDriveBase.getMotionConstants().getMaxRotationalVelocity()) < 0.01 * mecanumDriveBase.getMotionConstants().getMaxRotationalVelocity())
				.setCommand(
						new LambdaCommand()
								.setInit(() -> {
									endTime = getElapsedTime().seconds();
									endVelocity = velocity;
									turn = 0;
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
		telemetry.setAutoClear(true);
		startTime = getElapsedTime().seconds();
	}
	
	@Override
	public void loopEX() {
		double currentTime = getElapsedTime().seconds();
		telemetry.addData("max velocity", mecanumDriveBase.getMotionConstants().getMaxRotationalVelocity());
		velocities[velocityIndex] = Math.abs(mecanumDriveBase.getTracker().getPose2D().getTheta().getRadians() - mecanumDriveBase.getTracker().getPreviousPose2D().getTheta().getRadians()) / (currentTime - previousTime);
		velocityIndex++;
		velocityIndex %= velocitiesSize;
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
