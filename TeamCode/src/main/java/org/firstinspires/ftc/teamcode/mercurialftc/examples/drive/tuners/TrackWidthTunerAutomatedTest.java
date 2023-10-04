package org.firstinspires.ftc.teamcode.mercurialftc.examples.drive.tuners;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mercurialftc.examples.drive.MecanumDriveBase;
import org.mercurialftc.mercurialftc.scheduler.OpModeEX;
import org.mercurialftc.mercurialftc.scheduler.triggers.gamepadex.ContinuousInput;
import org.mercurialftc.mercurialftc.silversurfer.geometry.angle.AngleDegrees;
import org.mercurialftc.mercurialftc.silversurfer.geometry.angle.AngleRadians;
import org.mercurialftc.mercurialftc.silversurfer.geometry.Pose2D;

@TeleOp(name = "Track Width Tuner (Automated Test Version)")
public class TrackWidthTunerAutomatedTest extends OpModeEX {
	ElapsedTime elapsedTime;
	double measuredTurn;
	int fidelity;
	int fidelityI;
	double turn;
	private MecanumDriveBase mecanumDriveBase;
	private AngleRadians setPoint;
	private Pose2D measuredPose;
	private Pose2D previousPose;
	
	@Override
	public void registerSubsystems() {
		mecanumDriveBase = new MecanumDriveBase(
				this,
				new Pose2D(),
				new ContinuousInput(() -> 0),
				new ContinuousInput(() -> 0),
				new ContinuousInput(() -> turn)
		);
	}
	
	@Override
	public void initEX() {
		setPoint = new AngleDegrees(120).toAngleRadians();
		measuredPose = new Pose2D();
		previousPose = new Pose2D();
		elapsedTime = new ElapsedTime();
		
		fidelity = 5;
		fidelityI = 0;
		mecanumDriveBase.getTracker().reset();
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
		elapsedTime.reset();
		mecanumDriveBase.getTracker().reset();
	}
	
	@Override
	public void loopEX() {
		measuredPose = mecanumDriveBase.getTracker().getPose2D();
		
		measuredTurn += previousPose.getTheta().toAngleDegrees().findShortestDistance(measuredPose.getTheta());
		
		telemetry.addData("current odometry measured turn", measuredTurn);
		telemetry.addData("turn power", turn);
		
		
		previousPose = measuredPose;
		
		
		double error = measuredPose.getTheta().toAngleDegrees().findShortestDistance(setPoint); // in degrees
		
		double adjustedError = (error) / (Math.E * (Math.abs(error)) + 8);
		adjustedError *= 1.5;
		
		telemetry.addData("fidelityI", fidelityI);
		
		if (fidelityI < fidelity * 3 * 2 - 1) {
			if (Math.abs(error) < 40) {
				if (elapsedTime.seconds() > 0.3) {
					if (fidelityI < (fidelity * 3 - 1)) {
						setPoint = setPoint.add(new AngleDegrees(120)).toAngleRadians();
						fidelityI++;
					} else if ((fidelityI == fidelity * 3 - 1) == elapsedTime.seconds() > 5) {
						setPoint = setPoint.subtract(new AngleDegrees(120)).toAngleRadians();
						fidelityI++;
					}
				}
			} else {
				elapsedTime.reset();
			}
			
			turn = adjustedError;
		} else {
			if (Math.abs(error) < 0.2 && elapsedTime.seconds() > 2) {
				turn = 0;
			} else {
				turn = adjustedError;
			}
		}
		
	}
	
	@Override
	public void stopEX() {
	
	}
}
