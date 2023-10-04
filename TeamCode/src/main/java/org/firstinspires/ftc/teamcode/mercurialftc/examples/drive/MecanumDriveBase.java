package org.firstinspires.ftc.teamcode.mercurialftc.examples.drive;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.mercurialftc.mercurialftc.scheduler.OpModeEX;
import org.mercurialftc.mercurialftc.scheduler.commands.Command;
import org.mercurialftc.mercurialftc.scheduler.commands.LambdaCommand;
import org.mercurialftc.mercurialftc.scheduler.subsystems.Subsystem;
import org.mercurialftc.mercurialftc.scheduler.triggers.gamepadex.ContinuousInput;
import org.mercurialftc.mercurialftc.silversurfer.encoderticksconverter.EncoderTicksConverter;
import org.mercurialftc.mercurialftc.silversurfer.encoderticksconverter.Units;
import org.mercurialftc.mercurialftc.silversurfer.followable.motionconstants.MecanumMotionConstants;
import org.mercurialftc.mercurialftc.silversurfer.followable.Wave;
import org.mercurialftc.mercurialftc.silversurfer.follower.GVFWaveFollower;
import org.mercurialftc.mercurialftc.silversurfer.follower.MecanumArbFollower;
import org.mercurialftc.mercurialftc.silversurfer.follower.WaveFollower;
import org.mercurialftc.mercurialftc.silversurfer.geometry.Pose2D;
import org.mercurialftc.mercurialftc.silversurfer.geometry.Vector2D;
import org.mercurialftc.mercurialftc.silversurfer.geometry.angle.Angle;
import org.mercurialftc.mercurialftc.silversurfer.tracker.Tracker;
import org.mercurialftc.mercurialftc.silversurfer.tracker.TrackerConstants;
import org.mercurialftc.mercurialftc.silversurfer.tracker.TwoWheelTracker;
import org.mercurialftc.mercurialftc.silversurfer.voltageperformanceenforcer.VoltagePerformanceEnforcer;
import org.mercurialftc.mercurialftc.util.hardware.Encoder;
import org.mercurialftc.mercurialftc.util.hardware.IMU_EX;
import org.mercurialftc.mercurialftc.util.hardware.cachinghardwaredevice.CachingDcMotorEX;

public class MecanumDriveBase extends Subsystem {
	private final ContinuousInput x, y, t;
	private final Pose2D startPose;
	private final ElapsedTime waveTimer;
	private DcMotorEx fl, bl, br, fr;
	private VoltageSensor voltageSensor;
	private WaveFollower waveFollower;
	private MecanumArbFollower mecanumArbFollower;
	private Tracker tracker;
	private MecanumMotionConstants motionConstants;
	
	/**
	 * @param opModeEX  the opModeEX object to register against
	 * @param startPose the starting position
	 * @param x         the x controller
	 * @param y         the y controller
	 * @param t         the theta controller, positive turns clockwise
	 */
	public MecanumDriveBase(OpModeEX opModeEX, Pose2D startPose, ContinuousInput x, ContinuousInput y, ContinuousInput t) {
		super(opModeEX);
		this.startPose = startPose;
		this.x = x;
		this.y = y;
		this.t = t;
		this.waveTimer = new ElapsedTime();
	}
	
	
	@Override
	public void init() {
		// change the names of the motors as required
		fl = new CachingDcMotorEX(opModeEX.hardwareMap.get(DcMotorEx.class, "fl"));
		bl = new CachingDcMotorEX(opModeEX.hardwareMap.get(DcMotorEx.class, "bl"));
		br = new CachingDcMotorEX(opModeEX.hardwareMap.get(DcMotorEx.class, "br"));
		fr = new CachingDcMotorEX(opModeEX.hardwareMap.get(DcMotorEx.class, "fr"));
		
		// set the required motors to reverse
		fl.setDirection(DcMotorSimple.Direction.REVERSE);
		bl.setDirection(DcMotorSimple.Direction.FORWARD);
		br.setDirection(DcMotorSimple.Direction.FORWARD);
		fr.setDirection(DcMotorSimple.Direction.FORWARD);
		
		voltageSensor = opModeEX.hardwareMap.getAll(VoltageSensor.class).iterator().next();
		
		
		VoltagePerformanceEnforcer translationalYEnforcer = new VoltagePerformanceEnforcer(
				13.695, // volts
				0.5480339638343823, // amps
				1563.7458110091459 // in millimeters/second, you can use Units.UNIT.toMillimeters(value) to convert if you have measured some other way
		);
		
		VoltagePerformanceEnforcer translationalXEnforcer = new VoltagePerformanceEnforcer(
				13.975, // volts
				0.7686673487042971, // amps
				1123.465776909653 // in millimeters/second, you can use Units.UNIT.toMillimeters(value) to convert if you have measured some other way
		);
		
		VoltagePerformanceEnforcer translationalAngledEnforcer = new VoltagePerformanceEnforcer(
				13.931, // volts
				0.7242985361348729, // amps
				1008.932843434795711 // in millimeters/second, you can use Units.UNIT.toMillimeters(value) to convert if you have measured some other way
		);
		
		VoltagePerformanceEnforcer rotationalEnforcer = new VoltagePerformanceEnforcer(
				13.101, // volts
				0.6024987902362003, // amps
				3.9969453526299685 // radians/second
		);
		
		double currentVoltage = voltageSensor.getVoltage();
		
		// initial untuned motion constants, follow the tuning instructions to reach something like the example final results
//		motionConstants = new MecanumMotionConstants(
//				1,
//				1,
//				1,
//				1,
//				1,
//				1,
//				1,
//				1
//		);
		
		// example final results
		motionConstants = new MecanumMotionConstants(
				translationalYEnforcer.transformVelocity(currentVoltage),
				translationalXEnforcer.transformVelocity(currentVoltage),
				translationalAngledEnforcer.transformVelocity(currentVoltage),
				rotationalEnforcer.transformVelocity(currentVoltage),
				2229.3175544265337, // set to 1 to start with // in millimeters, you can use Units.UNIT.toMillimeters(value) to convert if you have measured some other way
				1577.2113772239454, // set to 1 to start with,
				1393.1682920895462,
				6.017290762302119
		);
		
		
		IMU_EX imu_ex = new IMU_EX(opModeEX.hardwareMap.get(IMU.class, "imu"), AngleUnit.RADIANS);
		imu_ex.initialize(new IMU.Parameters(
				new RevHubOrientationOnRobot(
						RevHubOrientationOnRobot.LogoFacingDirection.UP,
						RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
				)
		));
		
		// replace all, select the tracker which works best for you
		
		
		// insistent three wheeled tracker

//		tracker = new InsistentThreeWheelTracker(
//				startPose, // needs to be set to the starting pose, which should be the same pose as set for the wave builder
//				new TrackerConstants.ThreeWheelTrackerConstants(
//						Units.MILLIMETER,
//						377.26535034179676, // replace with your own measured constants
//						-172.5, // replace with your own measured constants,
//						1, // tune
//						1, // tune
//						new EncoderTicksConverter(8192/(Math.PI * 35), Units.MILLIMETER), // replace with your own measured constants
//						new EncoderTicksConverter(8192/(Math.PI * 35), Units.MILLIMETER), // replace with your own measured constants
//						new EncoderTicksConverter(8192/(Math.PI * 35), Units.MILLIMETER) // replace with your own measured constants
//				),
//				new Encoder(br).setDirection(Encoder.Direction.FORWARD), // check that each encoder increases in the positive direction, if not change their directions!
//				new Encoder(fr).setDirection(Encoder.Direction.REVERSE),
//				new Encoder(fl).setDirection(Encoder.Direction.FORWARD),
//				imu_ex
//		);
		
		// two wheeled tracker
		
		tracker = new TwoWheelTracker(
				startPose, // needs to be set to the starting pose, which should be the same pose as set for the wave builder
				new TrackerConstants.TwoWheelTrackerConstants(
						Units.MILLIMETER,
						-172.5, // replace with your own measured constants,
						(3000.0 / 2935.3374871243486), // tune
						(3000.0 / 2943.6016223376037), // tune
						new EncoderTicksConverter(8192 / (Math.PI * 35), Units.MILLIMETER), // replace with your own measured constants
						new EncoderTicksConverter(8192 / (Math.PI * 35), Units.MILLIMETER) // replace with your own measured constants
				),
				new Encoder(br).setDirection(Encoder.Direction.FORWARD), // check that each encoder increases in the positive direction, if not change their directions!
				new Encoder(fr).setDirection(Encoder.Direction.FORWARD),
				imu_ex
		);
		
		
		mecanumArbFollower = new MecanumArbFollower(
				motionConstants,
				tracker,
				fl, bl, br, fr
		);
		
		waveFollower = new GVFWaveFollower(
				mecanumArbFollower,
				tracker
		);

//		waveFollower = mecanumArbFollower;
		
		getTracker().reset(); // resets the encoders
		
		// set the motors to brake on 0 power
		fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		
		// sets the run modes
		fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
	}
	
	@Override
	public void periodic() {
		tracker.updatePose();
	}
	
	@Override
	public void defaultCommandExecute() {
		mecanumArbFollower.follow(
				new Vector2D(x.getValue(), y.getValue()),
				t.getValue()
		);
	}
	
	@Override
	public void close() {
	
	}
	
	/**
	 * an example command generator to set the robot to follow a prebuilt wave, for use during auto to prevent player inputs
	 * <p>requires {@link Command#queue()} to be called on this before it will run</p>
	 *
	 * @param wave the wave to follow
	 * @return the command to queue
	 */
	public Command followWave(Wave wave) {
		return new LambdaCommand()
				.setRequirements(this)
				.init(() -> {
					waveFollower.setWave(wave);
					waveTimer.reset();
				})
				.execute(() -> {
					waveFollower.update(waveTimer.seconds());
				})
				.finish(waveFollower::isFinished)
				.setInterruptable(true);
	}
	
	/**
	 * an example command generator to set the robot to follow a prebuilt wave, for use during teleop to allow player inputs to interrupt the movement
	 *
	 * @param wave the wave to follow
	 * @return the command to queue
	 */
	public Command followWaveInterruptible(Wave wave) {
		return new LambdaCommand()
				.setRequirements(this)
				.init(() -> {
					waveFollower.setWave(wave);
					waveTimer.reset();
				})
				.execute(() -> {
					waveFollower.update(waveTimer.seconds());
				})
				.finish(() -> waveFollower.isFinished() || x.getValue() != 0.0 || y.getValue() != 0.0 || t.getValue() != 0.0)
				.setInterruptable(true);
	}
	
	/**
	 * @return the drive base's position tracker
	 */
	public Tracker getTracker() {
		return tracker;
	}
	
	public void resetHeading() {
		tracker.resetHeading();
	}
	
	public void resetHeading(Angle heading) {
		tracker.resetHeading(heading);
	}
	
	public MecanumMotionConstants getMotionConstants() {
		return motionConstants;
	}
	
	public WaveFollower getWaveFollower() {
		return waveFollower;
	}
	
	public double getCurrent() {
		double result = 0.0;
		result += fl.getCurrent(CurrentUnit.AMPS);
		result += bl.getCurrent(CurrentUnit.AMPS);
		result += br.getCurrent(CurrentUnit.AMPS);
		result += fr.getCurrent(CurrentUnit.AMPS);
		return result / 4.0;
	}
	
	public VoltageSensor getVoltageSensor() {
		return voltageSensor;
	}
	
}
