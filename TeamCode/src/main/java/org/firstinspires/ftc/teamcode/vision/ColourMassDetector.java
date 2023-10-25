package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.mercurialftc.mercurialftc.scheduler.OpModeEX;
import org.mercurialftc.mercurialftc.scheduler.commands.Command;
import org.mercurialftc.mercurialftc.scheduler.commands.LambdaCommand;
import org.mercurialftc.mercurialftc.scheduler.subsystems.Subsystem;
import org.opencv.core.Scalar;

public class ColourMassDetector extends Subsystem {
	private final Scalar lower, upper;
	private final double minArea;
	private VisionPortal visionPortal;
	private ColourMassDetectionProcessor colourMassDetectionProcessor;
	
	public ColourMassDetector(
			OpModeEX opModeEX,
			Scalar lower,
			Scalar upper,
			double minArea
	) {
		super(opModeEX);
		this.lower = lower;
		this.upper = upper;
		this.minArea = minArea;
	}
	
	public ColourMassDetectionProcessor getColourMassDetectionProcessor() {
		return colourMassDetectionProcessor;
	}
	
	public VisionPortal getVisionPortal() {
		return visionPortal;
	}
	
	/**
	 * The code to be run when the OpMode is initialised.
	 */
	@Override
	public void init() {
		colourMassDetectionProcessor = new ColourMassDetectionProcessor(lower, upper, () -> minArea, () -> 213, () -> 426);
		visionPortal = new VisionPortal.Builder()
				.setCamera(opModeEX.hardwareMap.get(WebcamName.class, "Webcam 1"))
				.addProcessor(colourMassDetectionProcessor)
				.build();
	}
	
	/**
	 * The method that is ran at the start of every loop to facilitate encoder reads
	 * and any other calculations that need to be ran every loop regardless of the command
	 */
	@Override
	public void periodic() {
	
	}
	
	/**
	 * The default command run by a subsystem, will run every loop until a different command is scheduled over it
	 */
	@Override
	public void defaultCommandExecute() {
	
	}
	
	/**
	 * methods to be run when the subsystem is no longer used,
	 * for instance when the option to close the subsystem is implemented at the end of an OpMode,
	 * or when a new scheduler instance is forced.
	 */
	@Override
	public void close() {
		colourMassDetectionProcessor.close();
		visionPortal.close();
	}
	
	/**
	 * needs to be queued to run for INIT_LOOP
	 */
	public Command searchInitLoop() {
		return new LambdaCommand()
				.setRequirements(this) // make sure we require this subsystem!
				.setInterruptible(true)
				.setRunStates(OpModeEX.OpModeEXRunStates.INIT_LOOP) // causes this command to only run during init Loop, and end automatically outside of that
				.setInit(() -> {
					// when the command starts, start the live view and streaming, if not already
					if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING && visionPortal.getCameraState() != VisionPortal.CameraState.STARTING_STREAM) {
						visionPortal.resumeStreaming();
						visionPortal.resumeLiveView();
					}
				})
				.setEnd((interrupted) -> {
					// when the command ends, stop the live view and streaming
					if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING || visionPortal.getCameraState() == VisionPortal.CameraState.STARTING_STREAM) {
						visionPortal.stopLiveView();
						visionPortal.stopStreaming();
					}
				})
				.setFinish(() -> false); // this command "never" finishes (but will end once out of init loop)
	}
}
