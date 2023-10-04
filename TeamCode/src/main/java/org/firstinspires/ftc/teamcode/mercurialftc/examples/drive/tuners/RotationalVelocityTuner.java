package org.firstinspires.ftc.teamcode.mercurialftc.examples.drive.tuners;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mercurialftc.examples.drive.MecanumDriveBase;
import org.mercurialftc.mercurialftc.scheduler.OpModeEX;
import org.mercurialftc.mercurialftc.scheduler.commands.LambdaCommand;
import org.mercurialftc.mercurialftc.scheduler.triggers.gamepadex.ContinuousInput;
import org.mercurialftc.mercurialftc.silversurfer.geometry.Pose2D;
import org.mercurialftc.mercurialftc.silversurfer.tracker.Tracker;

@TeleOp(name = "Rotational Velocity Tuner")
public class RotationalVelocityTuner extends OpModeEX {
	private MecanumDriveBase mecanumDriveBase;
	private double previousTime;
	private double recordedVoltage;
	
	private double[] velocities;
	private double[] currents;
	private int index;
	private int range;
	
	private double averageVelocity;
	private double averageCurrent;
	
	
	private boolean running;
	
	
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
				new ContinuousInput(() -> running ? 1 : 0)
		);
	}
	
	/**
	 * should contain your regular init code
	 */
	@Override
	public void initEX() {
		mecanumDriveBase.getTracker().reset();
		range = 50;
		running = true;
	}
	
	/**
	 * registers triggers after the subsystem and regular init code,
	 * useful for organisation of your OpModeEX, but functionally no different to initialising them at the end of {@link #initEX()}
	 */
	@Override
	public void registerTriggers() {
		gamepadEX1().a().onPress(new LambdaCommand().init(() -> running = !running));
	}
	
	@Override
	public void init_loopEX() {
	
	}
	
	@Override
	public void startEX() {
		mecanumDriveBase.getTracker().reset();
		recordedVoltage = mecanumDriveBase.getVoltageSensor().getVoltage();
		velocities = new double[range];
		currents = new double[range];
		index = 0;
	}
	
	@Override
	public void loopEX() {
		double previousAverage = averageVelocity;
		double currentTime = getElapsedTime().seconds();
		Tracker tracker = mecanumDriveBase.getTracker();
		double rotationalVelocity = tracker.getPreviousPose2D().getTheta().findShortestDistance(tracker.getPose2D().getTheta()) / (currentTime - previousTime);
		
		
		if (running) {
			telemetry.addLine("press gamepad1 a to pause");
			
			velocities[index] = rotationalVelocity;
			currents[index] = mecanumDriveBase.getCurrent();
			index++;
			index %= range;
			
			for (int i = 0; i < range; i++) {
				averageVelocity += velocities[i];
				averageCurrent += currents[i];
			}
			averageVelocity /= range;
			averageCurrent /= range;
		} else {
			telemetry.addLine("press gamepad1 a to resume");
		}
		
		
		telemetry.addData("delta (stop when this is low)", Math.abs(averageVelocity - previousAverage));
		
		telemetry.addData("rotationalVelocity", averageVelocity);
		
		telemetry.addData("voltage", recordedVoltage);
		
		telemetry.addData("current", averageCurrent);
		
		
		previousTime = currentTime;
	}
	
	@Override
	public void stopEX() {
	
	}
}
