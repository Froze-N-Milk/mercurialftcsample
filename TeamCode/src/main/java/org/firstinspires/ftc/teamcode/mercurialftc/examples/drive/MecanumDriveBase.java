package org.firstinspires.ftc.teamcode.mercurialftc.examples.drive;

import androidx.annotation.NonNull;

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
import org.mercurialftc.mercurialftc.scheduler.bindings.gamepadex.DomainSupplier;
import org.mercurialftc.mercurialftc.silversurfer.encoderticksconverter.EncoderTicksConverter;
import org.mercurialftc.mercurialftc.silversurfer.encoderticksconverter.Units;
import org.mercurialftc.mercurialftc.silversurfer.followable.motionconstants.MecanumMotionConstants;
import org.mercurialftc.mercurialftc.silversurfer.followable.Wave;
import org.mercurialftc.mercurialftc.silversurfer.follower.ArbFollower;
import org.mercurialftc.mercurialftc.silversurfer.follower.GVFWaveFollower;
import org.mercurialftc.mercurialftc.silversurfer.follower.MecanumFollower;
import org.mercurialftc.mercurialftc.silversurfer.follower.ObstacleAvoidantFollower;
import org.mercurialftc.mercurialftc.silversurfer.follower.WaveFollower;
import org.mercurialftc.mercurialftc.silversurfer.geometry.Pose2D;
import org.mercurialftc.mercurialftc.silversurfer.geometry.Vector2D;
import org.mercurialftc.mercurialftc.silversurfer.geometry.angle.Angle;
import org.mercurialftc.mercurialftc.silversurfer.geometry.angle.AngleDegrees;
import org.mercurialftc.mercurialftc.silversurfer.geometry.angle.AngleRadians;
import org.mercurialftc.mercurialftc.silversurfer.geometry.obstaclemap.CenterStageObstacleMap;
import org.mercurialftc.mercurialftc.silversurfer.geometry.obstaclemap.ObstacleMap;
import org.mercurialftc.mercurialftc.silversurfer.tracker.InsistentThreeWheelTracker;
import org.mercurialftc.mercurialftc.silversurfer.tracker.ThreeWheelTracker;
import org.mercurialftc.mercurialftc.silversurfer.tracker.Tracker;
import org.mercurialftc.mercurialftc.silversurfer.tracker.TwoWheelTracker;
import org.mercurialftc.mercurialftc.silversurfer.tracker.WheeledTracker;
import org.mercurialftc.mercurialftc.silversurfer.tracker.WheeledTrackerConstants;
import org.mercurialftc.mercurialftc.silversurfer.voltageperformanceenforcer.VoltagePerformanceEnforcer;
import org.mercurialftc.mercurialftc.util.hardware.Encoder;
import org.mercurialftc.mercurialftc.util.hardware.IMU_EX;
import org.mercurialftc.mercurialftc.util.hardware.cachinghardwaredevice.CachingDcMotorEX;

public class MecanumDriveBase extends Subsystem {
	public static final double ONE_TILE = Units.INCH.toMillimeters(23.75);
	private final DomainSupplier x, y, t;
	private final Pose2D startPose;
	private final ElapsedTime waveTimer;
	private final Alliance alliance;
	private DcMotorEx fl, bl, br, fr;
	private VoltageSensor voltageSensor;
	private WaveFollower waveFollower;
	private ArbFollower arbFollower;
	private Tracker tracker;
	private MecanumMotionConstants motionConstants;
	private ObstacleMap obstacleMap;
	private double previousTime;
	
	/**
	 * @param opModeEX  the opModeEX object to register against
	 * @param startPose the starting position
	 * @param x         the x controller
	 * @param y         the y controller
	 * @param t         the theta controller, positive turns clockwise
	 */
	public MecanumDriveBase(OpModeEX opModeEX, Alliance alliance, Pose2D startPose, DomainSupplier x, DomainSupplier y, DomainSupplier t) {
		super(opModeEX);
		this.alliance = alliance;
		this.startPose = startPose;
		this.x = x;
		this.y = y;
		this.t = t;
		this.waveTimer = new ElapsedTime();
	}
	
	public MecanumDriveBase(OpModeEX opModeEX, Pose2D startPose, DomainSupplier x, DomainSupplier y, DomainSupplier t) {
		this(opModeEX, Alliance.RED, startPose, x, y, t);
	}
	
	public Pose2D getStartPose() {
		return startPose;
	}
	
	public ArbFollower getArbFollower() {
		return arbFollower;
	}
	
	public Alliance getAlliance() {
		return alliance;
	}
	
	public DomainSupplier getX() {
		return x;
	}
	
	public DomainSupplier getY() {
		return y;
	}
	
	public DomainSupplier getT() {
		return t;
	}
	
	public ObstacleMap getObstacleMap() {
		return obstacleMap;
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
		bl.setDirection(DcMotorSimple.Direction.REVERSE);
		br.setDirection(DcMotorSimple.Direction.FORWARD);
		fr.setDirection(DcMotorSimple.Direction.FORWARD);
		
		voltageSensor = opModeEX.hardwareMap.getAll(VoltageSensor.class).iterator().next();
		
		VoltagePerformanceEnforcer translationalYEnforcer = new VoltagePerformanceEnforcer(
				13.031,
				0.9747773604750232,
				1648.569587035565
		);
		
		VoltagePerformanceEnforcer translationalXEnforcer = new VoltagePerformanceEnforcer(
				12.987,
				1.769415503675299,
				1050.9146036238537
		);
		
		VoltagePerformanceEnforcer translationalAngledEnforcer = new VoltagePerformanceEnforcer(
				12.983,
				1.096447867768282,
				1222.6698612398357
		);
		
		VoltagePerformanceEnforcer rotationalEnforcer = new VoltagePerformanceEnforcer(
				13.096,
				0.9797365668388713,
				5.4708398890705805
		);
		
		double currentVoltage = voltageSensor.getVoltage();
		
		// replace accelerations
		motionConstants = new MecanumMotionConstants(
				translationalYEnforcer.transformVelocity(currentVoltage), // translational y velocity
				translationalXEnforcer.transformVelocity(currentVoltage), // translational x velocity
				translationalAngledEnforcer.transformVelocity(currentVoltage), // translational angled velocity
				rotationalEnforcer.transformVelocity(currentVoltage), // rotational velocity
				1406.4491188920347, // translational y acceleration
				1670.8888562062925, // translational x acceleration
				1311.448455610628, // translational angled acceleration
				9.943516004740639 // rotational acceleration
		);
		
		IMU_EX imu_ex = new IMU_EX(opModeEX.hardwareMap.get(IMU.class, "imu"), AngleUnit.RADIANS);
		imu_ex.initialize(new IMU.Parameters(
				new RevHubOrientationOnRobot(
						RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
						RevHubOrientationOnRobot.UsbFacingDirection.UP
				)
		));
		
		// two wheeled tracker
//		tracker = new TwoWheelTracker(
//				startPose, // needs to be set to the starting pose, which should be the same pose as set for the wave builder
//				new WheeledTrackerConstants.TwoWheeledTrackerConstants(
//						new Vector2D(-302.2 / 2.0, 416.9 / 2.0),
//						(3000.0 / 2953.6417571405955),
//						(3000.0 / 2961.3925604638556),
//						new EncoderTicksConverter(8192 / (Math.PI * 35), Units.MILLIMETER), // replace with your own measured constants
//						new EncoderTicksConverter(8192 / (Math.PI * 35), Units.MILLIMETER) // replace with your own measured constants
//				),
//				new Encoder(fl).setDirection(Encoder.Direction.FORWARD), // check that each encoder increases in the positive direction, if not change their directions!
//				new Encoder(bl).setDirection(Encoder.Direction.REVERSE),
//				imu_ex
//		);
		
		// three wheeled tracker
		tracker = new ThreeWheelTracker(
				startPose, // needs to be set to the starting pose, which should be the same pose as set for the wave builder
				new WheeledTrackerConstants.ThreeWheeledTrackerConstants(
						new Vector2D(-319.2 / 2.0, 0.0),
						(3000.0 / 2957.865463440085),
						(3000.0 / 2950.4555205066968),
						new EncoderTicksConverter(8192 / (Math.PI * 35), Units.MILLIMETER),
						new EncoderTicksConverter(8192 / (Math.PI * 35), Units.MILLIMETER), // replace with your own measured constants
						new EncoderTicksConverter(8192 / (Math.PI * 35), Units.MILLIMETER), // replace with your own measured constants
						392.93438
				),
				new Encoder(fl).setDirection(Encoder.Direction.FORWARD), // check that each encoder increases in the positive direction, if not change their directions!
				new Encoder(fr).setDirection(Encoder.Direction.REVERSE),
				new Encoder(bl).setDirection(Encoder.Direction.REVERSE)
		);
		
		MecanumFollower mecanumFollower = new MecanumFollower(
				motionConstants,
				tracker,
				fl, bl, br, fr
		);
		
		obstacleMap = new CenterStageObstacleMap(
				Units.MILLIMETER,
				ONE_TILE,
				200
		);
		
		arbFollower = new ObstacleAvoidantFollower(
				mecanumFollower,
				mecanumFollower,
				motionConstants,
				tracker,
				obstacleMap
		);

//		arbFollower = mecanumFollower;
		
		waveFollower = new GVFWaveFollower(
				(WaveFollower) arbFollower, // we know its a wave follower too, no harm in this
				motionConstants,
				tracker
		);

//		waveFollower = mecanumFollower;
		
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
		double currentTime = opModeEX.getElapsedTime().seconds();
		
		Vector2D vector2D = new Vector2D(x.getAsDouble(), y.getAsDouble()).rotate(alliance.getRotationAngle());
		
		arbFollower.follow(
				vector2D,
				t.getAsDouble(),
				currentTime - previousTime
		);
		
		previousTime = currentTime;
	}
	
	@Override
	public void close() {
	
	}
	
	/**
	 * an example command generator to set the robot to follow a prebuilt wave, for use during auto to prevent player inputs
	 * <p>requires {@link org.mercurialftc.mercurialftc.scheduler.commands.Command#queue()} to be called on this before it will run</p>
	 *
	 * @param wave the wave to follow
	 * @return the command to queue
	 */
	public Command followWave(Wave wave) {
		return new LambdaCommand()
				.setRequirements(this)
				.setInit(() -> {
					waveFollower.setWave(wave);
					waveTimer.reset();
				})
				.setExecute(() -> {
					waveFollower.update(waveTimer.seconds());
				})
				.setFinish(waveFollower::isFinished)
				.setInterruptible(true);
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
				.setInit(() -> {
					waveFollower.setWave(wave);
					waveTimer.reset();
				})
				.setExecute(() -> {
					waveFollower.update(waveTimer.seconds());
				})
				.setFinish(() -> waveFollower.isFinished() || x.getAsDouble() != 0.0 || y.getAsDouble() != 0.0 || t.getAsDouble() != 0.0)
				.setInterruptible(true);
	}
	
	/**
	 * @return the drive base's position tracker
	 */
	public WheeledTracker getTracker() {
		return (WheeledTracker) tracker;
	}
	
	public MecanumMotionConstants getMotionConstants() {
		return motionConstants;
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
	
	public enum Alliance {
		BLUE(new AngleDegrees(180)), RED(new AngleDegrees(0));
		private final AngleRadians rotationAngle;
		
		Alliance(@NonNull Angle rotationAngle) {
			this.rotationAngle = rotationAngle.toAngleRadians();
		}
		
		public Angle getRotationAngle() {
			return rotationAngle;
		}
	}
}
