package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Scalar;

@Disabled // remove this line to have this show up on your robot
@Autonomous
public class ColourMassDetectionOpMode extends OpMode {
	private VisionPortal visionPortal;
	private ColourMassDetectionProcessor colourMassDetectionProcessor;
	
	/**
	 * User-defined init method
	 * <p>
	 * This method will be called once, when the INIT button is pressed.
	 */
	@Override
	public void init() {
		// the current range set by lower and upper is the full range
		// HSV takes the form: (HUE, SATURATION, VALUE)
		// which means to select our colour, only need to change HUE
		// the domains are: ([0, 180], [0, 255], [0, 255])
		// this is tuned to detect red, so you will need to experiment to fine tune it for your robot
		// and experiment to fine tune it for blue
		Scalar lower = new Scalar(150, 100, 100); // the lower hsv threshold for your detection
		Scalar upper = new Scalar(180, 255, 255); // the upper hsv threshold for your detection
		double minArea = 100; // the minimum area for the detection to consider for your prop
		
		colourMassDetectionProcessor = new ColourMassDetectionProcessor(
				lower,
				upper,
				() -> minArea, // these are lambda methods, in case we want to change them while the match is running, for us to tune them or something
				() -> 213, // the left dividing line, in this case the left third of the frame
				() -> 426 // the left dividing line, in this case the right third of the frame
		);
		visionPortal = new VisionPortal.Builder()
				.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")) // the camera on your robot is named "Webcam 1" by default
				.addProcessor(colourMassDetectionProcessor)
				.build();
		
		// you may also want to take a look at some of the examples for instructions on
		// how to have a switchable camera (switch back and forth between two cameras)
		// or how to manually edit the exposure and gain, to account for different lighting conditions
		// these may be extra features for you to work on to ensure that your robot performs
		// consistently, even in different environments
	}
	
	/**
	 * User-defined init_loop method
	 * <p>
	 * This method will be called repeatedly during the period between when
	 * the init button is pressed and when the play button is pressed (or the
	 * OpMode is stopped).
	 * <p>
	 * This method is optional. By default, this method takes no action.
	 */
	@Override
	public void init_loop() {
		telemetry.addData("Currently Recorded Position", colourMassDetectionProcessor.getRecordedPropPosition());
		telemetry.addData("Camera State", visionPortal.getCameraState());
		telemetry.addData("Currently Detected Mass Center", "x: " + colourMassDetectionProcessor.getLargestContourX() + ", y: " + colourMassDetectionProcessor.getLargestContourY());
		telemetry.addData("Currently Detected Mass Area", colourMassDetectionProcessor.getLargestContourArea());
	}
	
	/**
	 * User-defined start method
	 * <p>
	 * This method will be called once, when the play button is pressed.
	 * <p>
	 * This method is optional. By default, this method takes no action.
	 * <p>
	 * Example usage: Starting another thread.
	 */
	@Override
	public void start() {
		// shuts down the camera once the match starts, we dont need to look any more
		if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
			visionPortal.stopLiveView();
			visionPortal.stopStreaming();
		}
		
		// gets the recorded prop position
		ColourMassDetectionProcessor.PropPositions recordedPropPosition = colourMassDetectionProcessor.getRecordedPropPosition();
		
		// now we can use recordedPropPosition to determine where the prop is! if we never saw a prop, your recorded position will be UNFOUND.
		// if it is UNFOUND, you can manually set it to any of the other positions to guess
		if (recordedPropPosition == ColourMassDetectionProcessor.PropPositions.UNFOUND) {
			recordedPropPosition = ColourMassDetectionProcessor.PropPositions.MIDDLE;
		}
		
		// now we can use recordedPropPosition in our auto code to modify where we place the purple and yellow pixels
		switch (recordedPropPosition) {
			case LEFT:
				// code to do if we saw the prop on the left
				break;
			case UNFOUND: // we can also just add the unfound case here to do fallthrough intstead of the overriding method above, whatever you prefer!
			case MIDDLE:
				// code to do if we saw the prop on the middle
				break;
			case RIGHT:
				// code to do if we saw the prop on the right
				break;
		}
	}
	
	/**
	 * User-defined loop method
	 * <p>
	 * This method will be called repeatedly during the period between when
	 * the play button is pressed and when the OpMode is stopped.
	 */
	@Override
	public void loop() {
	
	}
	
	/**
	 * User-defined stop method
	 * <p>
	 * This method will be called once, when this OpMode is stopped.
	 * <p>
	 * Your ability to control hardware from this method will be limited.
	 * <p>
	 * This method is optional. By default, this method takes no action.
	 */
	@Override
	public void stop() {
		// this closes down the portal when we stop the code, its good practice!
		colourMassDetectionProcessor.close();
		visionPortal.close();
	}
}
