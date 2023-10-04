package org.firstinspires.ftc.teamcode.mercurialftc.examples.drive.tuners;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.mercurialftc.examples.drive.MecanumDriveBase;
import org.mercurialftc.mercurialftc.scheduler.OpModeEX;
import org.mercurialftc.mercurialftc.scheduler.triggers.gamepadex.ContinuousInput;
import org.mercurialftc.mercurialftc.silversurfer.geometry.angle.AngleDegrees;
import org.mercurialftc.mercurialftc.silversurfer.geometry.angle.AngleRadians;
import org.mercurialftc.mercurialftc.silversurfer.geometry.Pose2D;
import org.mercurialftc.mercurialftc.util.hardware.ScheduledIMU_EX;

@TeleOp(name = "Track Width Tuner (Automated Version)")
public class TrackWidthTunerAutomated extends OpModeEX {
	ElapsedTime elapsedTime;
	double measuredTurn;
	int fidelity;
	int fidelityI;
	double lateralDistance;
	boolean finished;
	double turn;
	private MecanumDriveBase mecanumDriveBase;
	private ScheduledIMU_EX IMU_EX;
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
		IMU_EX = new ScheduledIMU_EX(this, new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)), AngleUnit.DEGREES);
	}
	
	@Override
	public void initEX() {
		setPoint = new AngleDegrees(120).toAngleRadians();
		measuredPose = new Pose2D();
		previousPose = new Pose2D();
		elapsedTime = new ElapsedTime();
		
		fidelity = 10;
		fidelityI = 0;
		finished = false;
		
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
		lateralDistance = mecanumDriveBase.getTracker().getTrackerConstants().getLateralDistance();
		mecanumDriveBase.getTracker().reset();
	}
	
	@Override
	public void loopEX() {
		measuredPose = mecanumDriveBase.getTracker().getPose2D();
		
		AngleDegrees IMUmeasuredTurn = IMU_EX.getImuEX().getYaw().toAngleDegrees();
		
		measuredTurn += previousPose.getTheta().toAngleDegrees().findShortestDistance(measuredPose.getTheta());
		
		telemetry.addData("current odometry measured turn", measuredTurn);
		
		previousPose = measuredPose;
		
		
		double error = IMUmeasuredTurn.findShortestDistance(setPoint); // in degrees
		
		double adjustedError = (error) / (Math.E * (Math.abs(error)) + 8);
		
		if (finished) {
			turn = 0;
			telemetry.addData("your track width is", ((measuredTurn * lateralDistance) / (fidelity * 360)));
		} else if (fidelityI < fidelity * 3 - 1) {
			if (Math.abs(error) < 40) {
				if (elapsedTime.seconds() > 0.3) {
					setPoint = setPoint.add(new AngleDegrees(120)).toAngleRadians();
					fidelityI++;
				}
			} else {
				elapsedTime.reset();
			}
			
			turn = adjustedError * mecanumDriveBase.getMotionConstants().getMaxRotationalVelocity();
		} else {
			if (Math.abs(error) < 0.2 && elapsedTime.seconds() > 2) {
				finished = true;
			} else if (!finished) {
				if (Math.abs(error) >= 0.2) {
					elapsedTime.reset();
				}
				turn = adjustedError * mecanumDriveBase.getMotionConstants().getMaxRotationalVelocity();
			}
		}
		telemetry.addData("error", error);
		telemetry.addData("setPoint", setPoint.getDegrees());
		
	}
	
	@Override
	public void stopEX() {
	
	}
}
