package org.firstinspires.ftc.teamcode.mercurialftc.examples.drive;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.mercurialftc.mercurialftc.scheduler.OpModeEX;
import org.mercurialftc.mercurialftc.scheduler.commands.CommandSignature;
import org.mercurialftc.mercurialftc.scheduler.triggers.gamepadex.ContinuousInput;
import org.mercurialftc.mercurialftc.silversurfer.encoderticksconverter.Units;
import org.mercurialftc.mercurialftc.silversurfer.followable.Wave;
import org.mercurialftc.mercurialftc.silversurfer.followable.WaveBuilder;
import org.mercurialftc.mercurialftc.silversurfer.geometry.AngleDegrees;
import org.mercurialftc.mercurialftc.silversurfer.geometry.Pose2D;

import java.util.Locale;

@TeleOp
public class DemoWaveFollowing extends OpModeEX {
	private final double ONE_SQUARE = Units.INCH.toMillimeters(23.75);
	private final Pose2D startPose = new Pose2D(-1.5 * ONE_SQUARE, -2.5 * ONE_SQUARE, new AngleDegrees(90));
	private Wave wave;
	private MecanumDriveBase mecanumDriveBase;
	
	/**
	 * called before {@link #initEX()}, solely for initialising all subsystems, ensures that they are registered with the correct {@link Scheduler}, and that their init methods will be run
	 */
	@Override
	public void registerSubsystems() {
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
				.splineTo(-0.5 * ONE_SQUARE, -0.5 * ONE_SQUARE, new AngleDegrees(90))
				.splineTo(2.5 * ONE_SQUARE, 0, new AngleDegrees(90))
				.waitFor(1.0)
				.splineTo(-0.5 * ONE_SQUARE, -0.5 * ONE_SQUARE, new AngleDegrees(90))
				.splineTo(-1.5 * ONE_SQUARE, -2.5 * ONE_SQUARE, new AngleDegrees(90));
		
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
		mecanumDriveBase.followWave(wave).queue();

//		telemetry.setAutoClear(false);
//
//		Telemetry.Line dataLine = telemetry.addLine();
//
//		Followable.Output output = new Followable.Output(new Vector2D(0, 0), 0, 0, new Pose2D(), new Pose2D());
//		getElapsedTime().reset();
//		while (!wave.update(getElapsedTime().seconds())) {
//			if (output != wave.getOutput()) {
//				output = wave.getOutput();
//				String data = String.format(Locale.ENGLISH, "x: %f, y: %f, r: %f, time: %f%n", output.getTranslationVector().getX(), output.getTranslationVector().getY(), output.getRotationalVelocity(), output.getCallbackTime());
//				dataLine.addData(String.valueOf(getElapsedTime().seconds()), data);
//			}
//		}
	}
	
	@Override
	public void loopEX() {
		telemetry.addData("wave output", String.format(Locale.ENGLISH, "x: %f, y: %f, r: %f", wave.getOutput().getTranslationVector().getX(), wave.getOutput().getTranslationVector().getY(), wave.getOutput().getRotationalVelocity()));
		telemetry.addData("wave target position", String.format(Locale.ENGLISH, "x: %f, y: %f, r: %f", wave.getOutput().getDestination().getX(), wave.getOutput().getDestination().getY(), wave.getOutput().getDestination().getTheta().getDegrees()));
		
		Telemetry.Line commands = telemetry.addLine("commands");
		for (CommandSignature command : getScheduler().getCommands()) {
//			commands.addData(command.getClass().getSimpleName(), Arrays.toString(command.getRequiredSubsystems().toArray()));
			commands.addData(command.getClass().getSimpleName(), command.finished());
		}
		
		telemetry.addData("is finished", mecanumDriveBase.getWaveFollower().isFinished());
		
		telemetry.addData("time", getElapsedTime().seconds());
	}
	
	@Override
	public void stopEX() {
	
	}
}
