package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.mercurialftc.mercurialftc.scheduler.OpModeEX;
import org.opencv.core.Scalar;

@Disabled // remove this line to have this show up on your robot
@TeleOp
public class ColourMassDetectionOpModeEX extends OpModeEX {
	private ColourMassDetector colourMassDetector;
	
	@Override
	public void registerSubsystems() {
		colourMassDetector = new ColourMassDetector(
				this,
				new Scalar(150, 100, 100),
				new Scalar(180, 255, 255),
				100
		);
	}
	
	@Override
	public void initEX() {
		// queues the command, it will end by itself when the init_loop ends
		colourMassDetector.searchInitLoop().queue();
	}
	
	@Override
	public void registerBindings() {
	
	}
	
	@Override
	public void init_loopEX() {
		// does telemetry stuff
		telemetry.addData("Currently Recorded Position", colourMassDetector.getColourMassDetectionProcessor().getRecordedPropPosition());
		telemetry.addData("Camera State", colourMassDetector.getVisionPortal().getCameraState());
		telemetry.addData("Currently Detected Mass Center", "x: " + colourMassDetector.getColourMassDetectionProcessor().getLargestContourX() + ", y: " + colourMassDetector.getColourMassDetectionProcessor().getLargestContourY());
		telemetry.addData("Currently Detected Mass Area", colourMassDetector.getColourMassDetectionProcessor().getLargestContourArea());
	}
	
	@Override
	public void startEX() {
	
	}
	
	@Override
	public void loopEX() {
		// does telemetry stuff
		telemetry.addData("Currently Recorded Position", colourMassDetector.getColourMassDetectionProcessor().getRecordedPropPosition());
		telemetry.addData("Camera State", colourMassDetector.getVisionPortal().getCameraState());
		telemetry.addData("Currently Detected Mass Center", "x: " + colourMassDetector.getColourMassDetectionProcessor().getLargestContourX() + ", y: " + colourMassDetector.getColourMassDetectionProcessor().getLargestContourY());
		telemetry.addData("Currently Detected Mass Area", colourMassDetector.getColourMassDetectionProcessor().getLargestContourArea());
	}
	
	@Override
	public void stopEX() {
		// no need to close things here like in the normal opmode, that gets handled automatically
	}
}
