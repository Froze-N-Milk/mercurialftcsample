package org.firstinspires.ftc.teamcode.mercurialftc.examples.drive.tuners;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mercurialftc.examples.drive.MecanumDriveBase;
import org.mercurialftc.mercurialftc.scheduler.OpModeEX;
import org.mercurialftc.mercurialftc.scheduler.commands.LambdaCommand;
import org.mercurialftc.mercurialftc.scheduler.bindings.gamepadex.DomainSupplier;
import org.mercurialftc.mercurialftc.silversurfer.geometry.Pose2D;
import org.mercurialftc.mercurialftc.silversurfer.geometry.Vector2D;
import org.mercurialftc.mercurialftc.silversurfer.tracker.Tracker;

@TeleOp(name = "Translational Y Velocity Tuner")
public class TranslationalYVelocityTuner extends OpModeEX {
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
	
	
	@Override
	public void registerSubsystems() {
		mecanumDriveBase = new MecanumDriveBase(
				this,
				new Pose2D(),
				new DomainSupplier(() -> 0),
				new DomainSupplier(() -> running ? 1 : 0),
				new DomainSupplier(() -> 0)
		);
	}
	
	@Override
	public void initEX() {
		mecanumDriveBase.getTracker().reset();
		range = 50;
		running = true;
	}
	
	@Override
	public void registerBindings() {
		gamepadEX1().a().onTrue(new LambdaCommand().setInit(() -> running = !running));
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
		Vector2D velocityVector = tracker.getDeltaPositionVector();
		
		double translationalVelocity = velocityVector.getMagnitude() / (currentTime - previousTime);
		
		
		if (running) {
			telemetry.addLine("press gamepad1 a to pause");
			
			velocities[index] = translationalVelocity;
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
		
		telemetry.addData("translationalVelocity", averageVelocity);
		
		telemetry.addData("voltage", recordedVoltage);
		
		telemetry.addData("current", averageCurrent);
		
		
		previousTime = currentTime;
	}
	
	@Override
	public void stopEX() {
	
	}
}
