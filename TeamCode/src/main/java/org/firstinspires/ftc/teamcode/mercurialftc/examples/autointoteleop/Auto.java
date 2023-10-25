package org.firstinspires.ftc.teamcode.mercurialftc.examples.autointoteleop;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.mercurialftc.examples.DemoSubsystem;
import org.mercurialftc.mercurialftc.scheduler.OpModeEX;
import org.mercurialftc.mercurialftc.scheduler.Scheduler;

import java.io.IOException;

/**
 * see readme.md in this directory
 */
@Disabled
// if you have the preselect option enabled, it will auto select teleop to run after this
@Autonomous(preselectTeleOp = "Teleop")
public class Auto extends OpModeEX {
	private DemoSubsystem demoSubsystem;
	
	@Override
	public void registerSubsystems() {
		demoSubsystem = new DemoSubsystem(this);
	}
	
	@Override
	public void initEX() {
	
	}
	
	@Override
	public void registerBindings() {
		// typically no trigger code is required in your auto!
	}
	
	@Override
	public void init_loopEX() {
	
	}
	
	@Override
	public void startEX() {
		// reset everything!
		
		// run your not asynchronous auto code here!
	}
	
	@Override
	public void loopEX() {
		// run your asynchronous auto code here!
	}
	
	@Override
	public void stopEX() {
		// we do NOT want to refresh the scheduler when we swap to teleop
		Scheduler.getConfigOptionsManager().updateValue(Scheduler.ConfigOptions.SCHEDULER_REFRESH_ENABLED.getOption(), false);
		
		
		try {
			Scheduler.getConfigOptionsManager().update(); // actually updates the setting we changed
		} catch (IOException e) {
			throw new RuntimeException(e);
		}
		// add each subsystem to the stored subsystems arraylist in the scheduler
		// we made sure not to wipe the scheduler, so we can get them back!
		getScheduler().storeSubsystem("my very own Demosubsystem", demoSubsystem); // we give it a unique name, so we can have multiple of one subsystem
//		getScheduler().storeSubsystem("demosubsystem2", demoSubsystem2); (this doesn't work bc i don't have more than one subsystem, but for demonstration's sake)
	}
}
