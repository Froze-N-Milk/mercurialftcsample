package org.firstinspires.ftc.teamcode.mercurialftc.examples;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.mercurialftc.examples.drive.MecanumDriveBase;
import org.mercurialftc.mercurialftc.scheduler.OpModeEX;
import org.mercurialftc.mercurialftc.scheduler.commands.Command;
import org.mercurialftc.mercurialftc.scheduler.subsystems.SubsystemInterface;
import org.mercurialftc.mercurialftc.silversurfer.geometry.Vector2D;
import org.mercurialftc.mercurialftc.silversurfer.geometry.angle.Angle;

import java.util.HashSet;
import java.util.Set;

@SuppressWarnings("unused")
public class DemoCommand implements Command {
	private final MecanumDriveBase mecanumDriveBase;
	private final DemoSubsystem demoSubsystem;
	private final Angle targetAngle;
	private final HashSet<SubsystemInterface> requirements;
	private final HashSet<OpModeEX.OpModeEXRunStates> runStates;
	private double previousTime;
	private double error, previousError;
	
	public DemoCommand(@NonNull MecanumDriveBase mecanumDriveBase, @NonNull DemoSubsystem demoSubsystem, Angle targetAngle) {
		this.mecanumDriveBase = mecanumDriveBase;
		this.demoSubsystem = demoSubsystem;
		this.targetAngle = targetAngle;
		
		requirements = new HashSet<>();
		requirements.add(demoSubsystem);
		requirements.add(mecanumDriveBase);
		
		runStates = new HashSet<>(1); // this is the defaultg
		runStates.add(OpModeEX.OpModeEXRunStates.LOOP);
	}
	
	// this is functionally equivalent to not having this block here
	@Override
	public boolean interruptable() {
		return true;
	}
	
	@Override
	public void initialise() {
		demoSubsystem.prepPID(1000);
		previousTime = mecanumDriveBase.opModeEX.getElapsedTime().seconds();
	}
	
	@Override
	public void execute() {
		demoSubsystem.PIDupdate();
		
		double currentTime = mecanumDriveBase.opModeEX.getElapsedTime().seconds();
		
		Vector2D vector2D = new Vector2D(mecanumDriveBase.getX().getAsDouble(), mecanumDriveBase.getY().getAsDouble()).rotate(mecanumDriveBase.getAlliance().getRotationAngle());
		
		error = mecanumDriveBase.getTracker().getPose2D().getTheta().findShortestDistance(targetAngle);
		
		mecanumDriveBase.getArbFollower().follow(
				vector2D,
				error / Math.PI,
				currentTime - previousTime
		);
		
		previousTime = currentTime;
	}
	
	@Override
	public void end(boolean interrupted) {
	
	}
	
	@Override
	public boolean finished() {
		boolean demoSubsystemFinished = Math.abs(demoSubsystem.getPosition() - demoSubsystem.getTargetPosition()) < 5;
		
		boolean mecanumDriveBaseFinished = Math.abs(error) < 0.03;
		
		return demoSubsystemFinished && mecanumDriveBaseFinished;
	}
	
	@Override
	public Set<SubsystemInterface> getRequiredSubsystems() {
		return requirements;
	}
	
	
	// this is functionally equivalent to not having this block here
	@Override
	public Set<OpModeEX.OpModeEXRunStates> getRunStates() {
		return runStates;
	}
}
