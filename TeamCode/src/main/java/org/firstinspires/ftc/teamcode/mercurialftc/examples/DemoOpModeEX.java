package org.firstinspires.ftc.teamcode.mercurialftc.examples;

import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.mercurialftc.examples.drive.MecanumDriveBase;
import org.mercurialftc.mercurialftc.scheduler.OpModeEX;
import org.mercurialftc.mercurialftc.scheduler.bindings.Binding;
import org.mercurialftc.mercurialftc.silversurfer.geometry.Pose2D;
import org.mercurialftc.mercurialftc.silversurfer.geometry.angle.AngleDegrees;

@SuppressWarnings("unused")
public class DemoOpModeEX extends OpModeEX {
	private DemoSubsystem demoSubsystem;
	private MecanumDriveBase mecanumDriveBase;
	private Binding<?> limitSwitch;
	
	@Override
	public void registerSubsystems() {
		demoSubsystem = new DemoSubsystem(this);
		mecanumDriveBase = new MecanumDriveBase(this, new Pose2D(), gamepadEX1().leftX(), gamepadEX1().leftY(), gamepadEX1().rightX().invert());
	}
	
	@Override
	public void initEX() {
		DigitalChannel limitSwitch = hardwareMap.get(DigitalChannel.class, "limitSwitch");
		limitSwitch.setMode(DigitalChannel.Mode.INPUT);
		
		this.limitSwitch = new Binding<>(() -> !limitSwitch.getState());
		this.limitSwitch.debounce(Binding.DebouncingType.LEADING_EDGE, 0.2);
	}
	
	@Override
	public void registerBindings() {
		gamepadEX2().dpad_down().onTrue(demoSubsystem.runTo(DemoSubsystem.Positions.ZERO));
		gamepadEX2().dpad_left().onTrue(demoSubsystem.runTo(DemoSubsystem.Positions.POS1));
		gamepadEX2().dpad_right().onTrue(demoSubsystem.runTo(DemoSubsystem.Positions.POS2));
		gamepadEX2().dpad_up().onTrue(demoSubsystem.runTo(DemoSubsystem.Positions.POS3));
		
		// DemoCommand is not a very good real-world example of a command
		gamepadEX1().y().onFalse(new DemoCommand(mecanumDriveBase, demoSubsystem, new AngleDegrees(90)));
		
		gamepadEX2().leftY().buildBinding().lessThan(-0.01).greaterThan(0.01).bind()
				.whileTrue(demoSubsystem.manualControl(gamepadEX1().leftY()));
		
		limitSwitch.onTrue(demoSubsystem.resetEncoders());
	}
	
	@Override
	public void init_loopEX() {
	
	}
	
	@Override
	public void startEX() {
	
	}
	
	@Override
	public void loopEX() {
	
	}
	
	@Override
	public void stopEX() {
	
	}
}
