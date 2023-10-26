package org.firstinspires.ftc.teamcode.mercurialftc.examples.drive;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.mercurialftc.mercurialftc.scheduler.OpModeEX;
import org.mercurialftc.mercurialftc.scheduler.commands.LambdaCommand;
import org.mercurialftc.mercurialftc.silversurfer.geometry.Pose2D;
import org.mercurialftc.mercurialftc.silversurfer.geometry.Vector2D;

@TeleOp(name = "Tracker test")
public class TrackerTest extends OpModeEX {
	private MecanumDriveBase mecanumDriveBase;
	
	@Override
	public void registerSubsystems() {
		mecanumDriveBase = new MecanumDriveBase(
				this,
				MecanumDriveBase.Alliance.RED,
				new Pose2D(),
				gamepadEX1().leftX(),
				gamepadEX1().leftY(),
				gamepadEX1().rightX().invert()
		);
	}
	
	@Override
	public void initEX() {
	}
	
	@Override
	public void registerBindings() {
		gamepadEX1().left_bumper().onTrue(
				new LambdaCommand().setInit(mecanumDriveBase.getTracker()::reset)
		);
	}
	
	@Override
	public void init_loopEX() {
	}
	
	@Override
	public void startEX() {
		mecanumDriveBase.getTracker().reset();
	}
	
	@Override
	public void loopEX() {
		telemetry.addData("position", mecanumDriveBase.getTracker().getPose2D());
		telemetry.addData("outputMagnitude", new Vector2D(gamepadEX1().leftX().getAsDouble(), gamepadEX1().leftY().getAsDouble()).getMagnitude());
		telemetry.addData("outputDirection", new Vector2D(gamepadEX1().leftX().getAsDouble(), gamepadEX1().leftY().getAsDouble()).getHeading().getDegrees());
	}
	
	@Override
	public void stopEX() {
	
	}
}
