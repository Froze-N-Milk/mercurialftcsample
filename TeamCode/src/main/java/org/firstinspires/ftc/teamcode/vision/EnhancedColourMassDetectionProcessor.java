package org.firstinspires.ftc.teamcode.vision;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Typeface;
import android.text.TextPaint;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.Locale;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.DoubleSupplier;

public class EnhancedColourMassDetectionProcessor implements VisionProcessor, CameraStreamSource {
	private final DoubleSupplier minArea, left, right;
	private final Scalar upper; // lower bounds for masking
	private final Scalar lower; // upper bounds for masking
	private final TextPaint textPaint;
	private final Paint linePaint;
	private final ArrayList<MatOfPoint> contours;
	private final Mat hierarchy = new Mat();
	private final Mat sel1 = new Mat(); // these facilitate capturing through 0
	private final Mat sel2 = new Mat();
	private final Mat mask = new Mat();
	private final AtomicReference<Bitmap> lastFrame = new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));
	private double mostSaturatedContourX;
	private double mostSaturatedContourY;
	private double mostSaturatedContourArea;
	private double mostSaturatedContourSaturation;
	private MatOfPoint mostSaturatedContour;
	private PropPositions previousPropPosition;
	private PropPositions recordedPropPosition = PropPositions.UNFOUND;
	
	
	/**
	 * Uses HSVs for the scalars
	 *
	 * @param lowerH  the lower masked bound, a H value from HSV, if lowerH is higher than higherH, it will wrap through 0
	 * @param upperH  the upper masked bound, a H value from HSV, if higherH is lower than lowerH, it will wrap through 0
	 * @param minArea the minimum area for a detected blob to be considered the prop
	 * @param left    the dividing point for the prop to be on the left
	 * @param right   the diving point for the prop to be on the right
	 */
	public EnhancedColourMassDetectionProcessor(double lowerH, double upperH, DoubleSupplier minArea, DoubleSupplier left, DoubleSupplier right) {
		this.contours = new ArrayList<>();
		this.lower = new Scalar(lowerH, 0, 0);
		this.upper = new Scalar(upperH, 255, 255);
		this.minArea = minArea;
		this.left = left;
		this.right = right;
		
		// setting up the paint for the text in the center of the box
		textPaint = new TextPaint();
		textPaint.setColor(Color.GREEN); // you may want to change this
		textPaint.setTextAlign(Paint.Align.CENTER);
		textPaint.setAntiAlias(true);
		textPaint.setTextSize(40); // or this
		textPaint.setTypeface(Typeface.DEFAULT_BOLD);
		
		// setting up the paint for the lines that comprise the box
		linePaint = new Paint();
		linePaint.setColor(Color.GREEN); // you may want to change this
		linePaint.setAntiAlias(true);
		linePaint.setStrokeWidth(10); // or this
		linePaint.setStrokeCap(Paint.Cap.ROUND);
		linePaint.setStrokeJoin(Paint.Join.ROUND);
	}
	
	@Override
	public void init(int width, int height, CameraCalibration calibration) {
		lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
		// this method comes with all VisionProcessors, we just don't need to do anything here, and you dont need to call it
	}
	
	/**
	 * @return the x position of the currently found most saturated contour in the range [0, camera width], or -1 if no largest contour has been determined
	 */
	public double getMostSaturatedContourX() {
		return mostSaturatedContourX;
	}
	
	/**
	 * @return the y position of the currently found most saturated contour in the range [0, camera height], or -1 if no largest contour has been determined
	 */
	public double getMostSaturatedContourY() {
		return mostSaturatedContourY;
	}
	
	/**
	 * @return the area of the currently found most saturated contour, or -1 if no largest contour has been determined
	 */
	public double getMostSaturatedContourArea() {
		return mostSaturatedContourArea;
	}
	
	/**
	 * @return the area of the currently found most saturated contour, or -1 if no largest contour has been determined
	 */
	public double getMostSaturatedContourSaturation() {
		return mostSaturatedContourSaturation;
	}
	
	@Override
	public Object processFrame(Mat frame, long captureTimeNanos) {
		
		// this method processes the image (frame) taken by the camera, and tries to find a suitable prop
		// you dont need to call it
		
		// this converts the frame from RGB to HSV, which is supposed to be better for doing colour blob detection
		Imgproc.cvtColor(frame, frame, Imgproc.COLOR_RGB2HSV);
		// thats why you need to give your scalar upper and lower bounds as HSV values
		
		if (upper.val[0] < lower.val[0]) {
			// makes new scalars for the upper [upper, 0] detection, places the result in sel1
			Core.inRange(frame, new Scalar(upper.val[0], lower.val[1], lower.val[2]), new Scalar(0, upper.val[1], upper.val[2]), sel1);
			// makes new scalars for the lower [0, lower] detection, places the result in sel2
			Core.inRange(frame, new Scalar(0, lower.val[1], lower.val[2]), new Scalar(lower.val[0], upper.val[1], upper.val[2]), sel2);
			
			// combines the selections
			Core.bitwise_or(sel1, sel2, mask);
		} else {
			// this process is simpler if we are not trying to wrap through 0
			// this method makes the colour image black and white, with everything between your upper and lower bound values as white, and everything else black
			Core.inRange(frame, lower, upper, mask);
		}
		
		// apply the mask to frame
		Core.bitwise_and(frame, mask, frame);
		
		// this empties out the list of found contours, otherwise we would keep all the old ones, read on to find out more about contours!
		contours.clear();
		
		// this finds the contours, which are borders between black and white, and tries to simplify them to make nice outlines around potential objects
		// this basically helps us to find all the shapes/outlines of objects that exist within our colour range
		Imgproc.findContours(frame, contours, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
		
		// this sets up our largest contour area to be 0
		mostSaturatedContourArea = -1;
		mostSaturatedContourSaturation = -1;
		
		// and our currently found largest contour to be null
		mostSaturatedContour = null;
		
		// gets the current minimum area from min area
		double minArea = this.minArea.getAsDouble();
		
		// finds the largest contour!
		// for each contour we found before we loop over them, calculate their area,
		// and then if our area is larger than our minimum area, and our currently found largest area
		// it stores the contour as our largest contour and the area as our largest area
		for (MatOfPoint contour : contours) {
			double meanSaturation = Core.mean(contour).val[1];
			double area = Imgproc.contourArea(contour);
			if (meanSaturation > mostSaturatedContourSaturation && area > minArea) {
				mostSaturatedContour = contour;
				mostSaturatedContourSaturation = meanSaturation;
				mostSaturatedContourArea = area;
			}
		}
		
		
		// sets up the center points of our largest contour to be -1 (offscreen)
		mostSaturatedContourX = mostSaturatedContourY = -1;
		
		// if we found it, calculates the actual centers
		if (mostSaturatedContour != null) {
			Moments moment = Imgproc.moments(mostSaturatedContour);
			mostSaturatedContourX = (moment.m10 / moment.m00);
			mostSaturatedContourY = (moment.m01 / moment.m00);
		}
		
		// determines the current prop position, using the left and right dividers we gave earlier
		// if we didn't find any contours which were large enough, sets it to be unfound
		PropPositions propPosition;
		if (mostSaturatedContour == null) {
			propPosition = PropPositions.UNFOUND;
		} else if (mostSaturatedContourX < left.getAsDouble()) {
			propPosition = PropPositions.LEFT;
		} else if (mostSaturatedContourX > right.getAsDouble()) {
			propPosition = PropPositions.RIGHT;
		} else {
			propPosition = PropPositions.MIDDLE;
		}
		
		// if we have found a new prop position, and it is not unfound, updates the recorded position,
		// this makes sure that if our camera is playing up, we only need to see the prop in the correct position
		// and we will hold onto it
		if (propPosition != previousPropPosition && propPosition != PropPositions.UNFOUND) {
			recordedPropPosition = propPosition;
		}
		
		// updates the previous prop position to help us check for changes
		previousPropPosition = propPosition;

//		Imgproc.drawContours(frame, contours, -1, colour);
		
		// returns back the edited image, don't worry about this too much
		Bitmap b = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565);
		Utils.matToBitmap(frame, b);
		lastFrame.set(b);
		return frame;
	}
	
	@Override
	public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
		// this method draws the rectangle around the largest contour and puts the current prop position into that rectangle
		// you don't need to call it

//		for (MatOfPoint contour : contours) {
//			Rect rect = Imgproc.boundingRect(contour);
//			canvas.drawLines(new float[]{rect.x * scaleBmpPxToCanvasPx, rect.y * scaleBmpPxToCanvasPx, (rect.x + rect.width) * scaleBmpPxToCanvasPx, (rect.y + rect.height) * scaleBmpPxToCanvasPx}, textPaint);
//		}
		
		// if the contour exists, draw a rectangle around it and put its position in the middle of the rectangle
		if (mostSaturatedContour != null) {
			Rect rect = Imgproc.boundingRect(mostSaturatedContour);
			
			float[] points = {rect.x * scaleBmpPxToCanvasPx, rect.y * scaleBmpPxToCanvasPx, (rect.x + rect.width) * scaleBmpPxToCanvasPx, (rect.y + rect.height) * scaleBmpPxToCanvasPx};
			
			canvas.drawLine(points[0], points[1], points[0], points[3], linePaint);
			canvas.drawLine(points[0], points[1], points[2], points[1], linePaint);
			
			canvas.drawLine(points[0], points[3], points[2], points[3], linePaint);
			canvas.drawLine(points[2], points[1], points[2], points[3], linePaint);
			
			String text = String.format(Locale.ENGLISH, "%s", recordedPropPosition.toString());
			
			canvas.drawText(text, (float) mostSaturatedContourX * scaleBmpPxToCanvasPx, (float) mostSaturatedContourY * scaleBmpPxToCanvasPx, textPaint);
		}
	}
	
	/**
	 * @return the last found prop position, if none have been found, returns {@link PropPositions#UNFOUND}
	 */
	public PropPositions getRecordedPropPosition() {
		return recordedPropPosition;
	}
	
	// returns the largest contour if you want to get information about it
	public MatOfPoint getMostSaturatedContour() {
		return mostSaturatedContour;
	}
	
	@Override
	protected void finalize() throws Throwable {
		close();
		super.finalize();
	}
	
	public void close() {
		hierarchy.release();
		sel1.release();
		sel2.release();
		mask.release();
	}
	
	@Override
	public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
		continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
	}
	
	// the enum that stores the 4 possible prop positions
	public enum PropPositions {
		LEFT,
		MIDDLE,
		RIGHT,
		UNFOUND;
	}
}
