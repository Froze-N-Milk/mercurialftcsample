package org.firstinspires.ftc.teamcode.mercurialftc.examples.drive;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.mercurialftc.mercurialftc.scheduler.OpModeEX;
import org.mercurialftc.mercurialftc.silversurfer.geometry.AngleDegrees;
import org.mercurialftc.mercurialftc.silversurfer.geometry.Pose2D;
import org.mercurialftc.mercurialftc.silversurfer.geometry.Vector2D;

@TeleOp(name = "Tracker test")
public class TrackerTest extends OpModeEX {
	private MecanumDriveBase mecanumDriveBase;
	
	@Override
	public void registerSubsystems() {
		mecanumDriveBase = new MecanumDriveBase(
				this,
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
	public void registerTriggers() {
	
	}
	
	@Override
	public void init_loopEX() {
	}
	
	@Override
	public void startEX() {
		telemetry.setAutoClear(true);
		mecanumDriveBase.getTracker().reset();
	}
	
	@Override
	public void loopEX() {
		telemetry.addData("X", mecanumDriveBase.getTracker().getPose2D().getX());
		telemetry.addData("Y", mecanumDriveBase.getTracker().getPose2D().getY());
		telemetry.addData("THETA (degrees)", mecanumDriveBase.getTracker().getPose2D().getTheta().getDegrees());
		telemetry.addData("total displacement", new Vector2D(mecanumDriveBase.getTracker().getPose2D().getX(), mecanumDriveBase.getTracker().getPose2D().getY()).getMagnitude());
	}
	
	@Override
	public void stopEX() {
	
	}
}
