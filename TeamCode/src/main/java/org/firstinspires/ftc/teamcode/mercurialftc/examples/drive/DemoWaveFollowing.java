package org.firstinspires.ftc.teamcode.mercurialftc.examples.drive;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.mercurialftc.mercurialftc.scheduler.OpModeEX;
import org.mercurialftc.mercurialftc.scheduler.Scheduler;
import org.mercurialftc.mercurialftc.scheduler.commands.Command;
import org.mercurialftc.mercurialftc.scheduler.triggers.gamepadex.ContinuousInput;
import org.mercurialftc.mercurialftc.silversurfer.encoderticksconverter.Units;
import org.mercurialftc.mercurialftc.silversurfer.followable.Wave;
import org.mercurialftc.mercurialftc.silversurfer.followable.WaveBuilder;
import org.mercurialftc.mercurialftc.silversurfer.geometry.angle.AngleDegrees;
import org.mercurialftc.mercurialftc.silversurfer.geometry.Pose2D;

@Autonomous
public class DemoWaveFollowing extends OpModeEX {
	private final double ONE_SQUARE = Units.INCH.toMillimeters(23.75);
	
	private Pose2D startPose;
	private Wave wave;
	private MecanumDriveBase mecanumDriveBase;
	
	/**
	 * called before {@link #initEX()}, solely for initialising all subsystems, ensures that they are registered with the correct {@link Scheduler}, and that their init methods will be run
	 */
	@Override
	public void registerSubsystems() {
		startPose = new Pose2D(-1.5 * ONE_SQUARE, -2.5 * ONE_SQUARE, new AngleDegrees(0));
		mecanumDriveBase = new MecanumDriveBase(
				this,
				startPose,
				new ContinuousInput(() -> 0),
				new ContinuousInput(() -> 0),
				new ContinuousInput(() -> 0)
		);
	}
	
	/**
	 * should contain your regular init code
	 */
	@Override
	public void initEX() {
		WaveBuilder waveBuilder = new WaveBuilder(
				startPose,
				Units.MILLIMETER,
				mecanumDriveBase.getMotionConstants()
		)
				.scaleTranslationVelocity(0.8)
				.scaleTranslationAcceleration(0.8)
				.splineTo(-1.5 * ONE_SQUARE, -1.5 * ONE_SQUARE, new AngleDegrees(0))
				.splineTo(0, 0, new AngleDegrees(0))
				.waitFor(1)
				.splineTo(-1.5 * ONE_SQUARE, -1.5 * ONE_SQUARE, new AngleDegrees(0))
				.splineTo(-1.5 * ONE_SQUARE, -2.5 * ONE_SQUARE, new AngleDegrees(0));
		
		wave = waveBuilder.build();
	}
	
	/**
	 * registers triggers after the subsystem and regular init code,
	 * useful for organisation of your OpModeEX, but functionally no different to initialising them at the end of {@link #initEX()}
	 */
	@Override
	public void registerTriggers() {
	
	}
	
	@Override
	public void init_loopEX() {
	
	}
	
	@Override
	public void startEX() {
		Command following = mecanumDriveBase.followWave(wave);
		
		following.queue();
	}
	
	@Override
	public void loopEX() {
//		telemetry.addData("wave output", String.format(Locale.ENGLISH, "%s, r: %f", wave.getOutput().getTranslationVector(), wave.getOutput().getRotationalVelocity()));
		telemetry.addData("target pose", wave.getOutput().getPosition());
		telemetry.addData("pose", mecanumDriveBase.getTracker().getPose2D());
	}
	
	@Override
	public void stopEX() {
	
	}
}
