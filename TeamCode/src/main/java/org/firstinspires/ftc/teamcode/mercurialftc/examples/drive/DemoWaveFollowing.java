package org.firstinspires.ftc.teamcode.mercurialftc.examples.drive;

import static org.firstinspires.ftc.teamcode.mercurialftc.examples.drive.MecanumDriveBase.ONE_TILE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.mercurialftc.mercurialftc.scheduler.OpModeEX;
import org.mercurialftc.mercurialftc.scheduler.commands.Command;
import org.mercurialftc.mercurialftc.scheduler.bindings.gamepadex.DomainSupplier;
import org.mercurialftc.mercurialftc.silversurfer.encoderticksconverter.Units;
import org.mercurialftc.mercurialftc.silversurfer.followable.Wave;
import org.mercurialftc.mercurialftc.silversurfer.followable.WaveBuilder;
import org.mercurialftc.mercurialftc.silversurfer.geometry.angle.AngleDegrees;
import org.mercurialftc.mercurialftc.silversurfer.geometry.Pose2D;

@Autonomous
public class DemoWaveFollowing extends OpModeEX {
	private Pose2D startPose;
	private Wave wave, wave1, wave2;
	private MecanumDriveBase mecanumDriveBase;
	
	@Override
	public void registerSubsystems() {
		startPose = new Pose2D(1.5 * ONE_TILE, 2.5 * ONE_TILE, new AngleDegrees(180));
		mecanumDriveBase = new MecanumDriveBase(
				this,
				startPose,
				new DomainSupplier(() -> 0),
				new DomainSupplier(() -> 0),
				new DomainSupplier(() -> 0)
		);
	}
	
	@Override
	public void initEX() {
		WaveBuilder waveBuilder1 = new WaveBuilder(
				startPose,
				Units.MILLIMETER,
				mecanumDriveBase.getMotionConstants(),
				mecanumDriveBase.getObstacleMap()
		)
				.scaleTranslationVelocity(1)
				.scaleTranslationAcceleration(1)
				.lineTo(startPose.getX(), startPose.getY() - 2 * ONE_TILE, new AngleDegrees(180 + 60))
				.turnTo(new AngleDegrees(180 + 120))
				.waitFor(0.5)
				.lineTo(startPose.getX() - 1 * ONE_TILE, startPose.getY() - 1 * ONE_TILE, new AngleDegrees(180 + 180))
				.turnTo(new AngleDegrees(180 + 240))
				.waitFor(0.5)
				.lineTo(startPose.getX(), startPose.getY(), new AngleDegrees(180 + 300))
				.turn(new AngleDegrees(60));
		
		WaveBuilder waveBuilder2 = new WaveBuilder(
				startPose,
				Units.MILLIMETER,
				mecanumDriveBase.getMotionConstants(),
				mecanumDriveBase.getObstacleMap()
		)
				.scaleTranslationVelocity(1)
				.scaleTranslationAcceleration(1)
				.splineTo(0.5 * ONE_TILE, 1.5 * ONE_TILE, new AngleDegrees(180))
				.splineTo(-1.5 * ONE_TILE, 1.5 * ONE_TILE, new AngleDegrees(180))
				.splineTo(-2 * ONE_TILE, 0, new AngleDegrees(90))
				.waitFor(1)
				.splineTo(-1.5 * ONE_TILE, 1.5 * ONE_TILE, new AngleDegrees(90))
				.splineTo(0.5 * ONE_TILE, 1.5 * ONE_TILE, new AngleDegrees(90))
				.splineTo(1.5 * ONE_TILE, 2.5 * ONE_TILE, new AngleDegrees(180));
		
		wave1 = waveBuilder1.build();
		wave2 = waveBuilder2.build();
	}
	
	@Override
	public void registerBindings() {
	
	}
	
	@Override
	public void init_loopEX() {
	
	}
	
	@Override
	public void startEX() {
		wave = wave1.concat(wave2);
		Command following = mecanumDriveBase.followWave(wave);
		mecanumDriveBase.getTracker().reset();
		following.queue();
	}
	
	@Override
	public void loopEX() {
		telemetry.addData("target pose", wave.getOutput().getPosition());
		telemetry.addData("pose", mecanumDriveBase.getTracker().getPose2D());
	}
	
	@Override
	public void stopEX() {
	
	}
}
