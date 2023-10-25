package org.firstinspires.ftc.teamcode.mercurialftc.examples.autointoteleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mercurialftc.examples.DemoSubsystem;
import org.mercurialftc.mercurialftc.scheduler.OpModeEX;
import org.mercurialftc.mercurialftc.scheduler.Scheduler;

import java.io.IOException;

/**
 * see readme.md in this directory
 */
@Disabled
@TeleOp
public class Teleop extends OpModeEX {
	private DemoSubsystem demoSubsystem;
	
	@Override
	public void registerSubsystems() {
		demoSubsystem = (DemoSubsystem) getScheduler().getStoredSubsystem("my very own Demosubsystem"); // we get back the first stored subsystem, and cast it back to our known type, this also removes it from the stored subsystems
//		demoSubsystem2 = (DemoSubsystem2) getScheduler().getStoredSubsystem("demosubsystem2"); // we could get back the second stored subsystem, and cast it back to our known type (but we don't have any more subsystems in this case)
	}
	
	@Override
	public void initEX() {
		Scheduler.getConfigOptionsManager().updateValue(Scheduler.ConfigOptions.SCHEDULER_REFRESH_ENABLED.getOption(), true); // after this OpModeEX runs we might want to go back to resetting the scheduler
	}
	
	@Override
	public void registerBindings() {
		// register all your driver and operator control triggers here
		gamepadEX1().a().whileTrue(demoSubsystem.getDefaultCommand()); // this doesn't actually do anything as this command is guaranteed to be running, and we aren't running anything else but, for demonstration's sake
	}
	
	@Override
	public void init_loopEX() {
	
	}
	
	@Override
	public void startEX() {
	
	}
	
	@Override
	public void loopEX() {
		// your code!
		
	}
	
	@Override
	public void stopEX() {
		try {
			Scheduler.getConfigOptionsManager().update(); // actually updates the setting we changed
		} catch (IOException e) {
			throw new RuntimeException(e);
		}
		
		// no need to do anything specific here
	}
}
