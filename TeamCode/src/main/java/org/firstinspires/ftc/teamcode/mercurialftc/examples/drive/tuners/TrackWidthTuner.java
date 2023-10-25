package org.firstinspires.ftc.teamcode.mercurialftc.examples.drive.tuners;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mercurialftc.examples.drive.MecanumDriveBase;
import org.mercurialftc.mercurialftc.scheduler.OpModeEX;
import org.mercurialftc.mercurialftc.scheduler.bindings.gamepadex.DomainSupplier;
import org.mercurialftc.mercurialftc.silversurfer.geometry.Pose2D;
import org.mercurialftc.mercurialftc.silversurfer.tracker.WheeledTrackerConstants;

@TeleOp(name = "Track Width Tuner")
public class TrackWidthTuner extends OpModeEX {
	double measuredTurn;
	int fidelity;
	int fidelityI;
	double lateralDistance;
	private MecanumDriveBase mecanumDriveBase;
	private Pose2D measuredPose;
	private Pose2D previousPose;
	private ElapsedTime elapsedTime;
	
	@Override
	public void registerSubsystems() {
		mecanumDriveBase = new MecanumDriveBase(
				this,
				new Pose2D(),
				new DomainSupplier(() -> 0),
				new DomainSupplier(() -> 0),
				new DomainSupplier(() -> 0)
		);
	}
	
	@Override
	public void initEX() {
		measuredPose = new Pose2D();
		previousPose = new Pose2D();
		elapsedTime = new ElapsedTime();
		
		fidelity = 5;
		fidelityI = 0;
	}
	
	@Override
	public void registerBindings() {
	
	}
	
	@Override
	public void init_loopEX() {
	}
	
	@Override
	public void startEX() {
		telemetry.setAutoClear(true);
		elapsedTime.reset();
		lateralDistance = ((WheeledTrackerConstants.ThreeWheeledTrackerConstants) mecanumDriveBase.getTracker().getTrackerConstants()).getTrackWidth();
		mecanumDriveBase.getTracker().reset();
	}
	
	@Override
	public void loopEX() {
		measuredPose = mecanumDriveBase.getTracker().getPose2D();
		
		measuredTurn += previousPose.getTheta().toAngleDegrees().findShortestDistance(measuredPose.getTheta());
		
		telemetry.addData("current odometry measured turn", measuredTurn);
		
		telemetry.addLine();
		telemetry.addLine("Instructions");
		telemetry.addLine();
		telemetry.addLine("turn the robot clockwise n times, I recommend that n is 5 or 10");
		telemetry.addLine("your new track width will be equal to:");
		telemetry.addLine((lateralDistance * measuredTurn) + " / (n * 360)");
		telemetry.addData("if n was 1, your new track width is", ((lateralDistance * measuredTurn) / (360)));
		telemetry.addData("if n was 5, your new track width is", ((lateralDistance * measuredTurn) / (5 * 360)));
		telemetry.addData("if n was 10, your new track width is", ((lateralDistance * measuredTurn) / (10 * 360)));
		
		telemetry.addLine("Then run the automated track width tuner test to check that it accurately turns 5 times");
		
		previousPose = measuredPose;
	}
	
	@Override
	public void stopEX() {
	
	}
}
