package org.firstinspires.ftc.teamcode.vision;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Typeface;
import android.text.TextPaint;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.Locale;
import java.util.function.DoubleSupplier;

public class ColourMassDetectionProcessor implements VisionProcessor {
	private final DoubleSupplier minArea, left, right;
	private final Scalar upper; // lower bounds for masking
	private final Scalar lower; // upper bounds for masking
	private final TextPaint textPaint;
	private final Paint linePaint;
	private final ArrayList<MatOfPoint> contours;
	private final Mat hierarchy = new Mat();
	private double largestContourX;
	private double largestContourY;
	private double largestContourArea;
	private MatOfPoint largestContour;
	private PropPositions previousPropPosition;
	private PropPositions recordedPropPosition = PropPositions.UNFOUND;
	
	/**
	 * Uses HSVs for the scalars
	 *
	 * @param lower   the lower masked bound, a three a value scalar in the form of a HSV
	 * @param upper   the upper masked bound, a three a value scalar in the form of a HSV
	 * @param minArea the minimum area for a detected blob to be considered the prop
	 * @param left    the dividing point for the prop to be on the left
	 * @param right   the diving point for the prop to be on the right
	 */
	public ColourMassDetectionProcessor(@NonNull Scalar lower, @NonNull Scalar upper, DoubleSupplier minArea, DoubleSupplier left, DoubleSupplier right) {
		this.contours = new ArrayList<>();
		this.lower = lower;
		this.upper = upper;
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
		// this method comes with all VisionProcessors, we just don't need to do anything here, and you dont need to call it
	}
	
	/**
	 * @return the x position of the currently found largest contour in the range [0, camera width], or -1 if no largest contour has been determined
	 */
	public double getLargestContourX() {
		return largestContourX;
	}
	
	/**
	 * @return the y position of the currently found largest contour in the range [0, camera height], or -1 if no largest contour has been determined
	 */
	public double getLargestContourY() {
		return largestContourY;
	}
	
	/**
	 * @return the area of the currently found largest contour, or -1 if no largest contour has been determined
	 */
	public double getLargestContourArea() {
		return largestContourArea;
	}
	
	@Override
	public Object processFrame(Mat frame, long captureTimeNanos) {
		// this method processes the image (frame) taken by the camera, and tries to find a suitable prop
		// you dont need to call it
		
		// this converts the frame from RGB to HSV, which is supposed to be better for doing colour blob detection
		Imgproc.cvtColor(frame, frame, Imgproc.COLOR_RGB2HSV);
		// thats why you need to give your scalar upper and lower bounds as HSV values
		
		// this method makes the colour image black and white, with everything between your upper and lower bound values as white, and everything else black
		Core.inRange(frame, lower, upper, frame);
		
		// this empties out the list of found contours, otherwise we would keep all the old ones, read on to find out more about contours!
		contours.clear();
		
		// this finds the contours, which are borders between black and white, and tries to simplify them to make nice outlines around potential objects
		// this basically helps us to find all the shapes/outlines of objects that exist within our colour range
		Imgproc.findContours(frame, contours, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
		
		// this sets up our largest contour area to be 0
		largestContourArea = -1;
		// and our currently found largest contour to be null
		largestContour = null;
		
		// gets the current minimum area from min area
		double minArea = this.minArea.getAsDouble();
		
		// finds the largest contour!
		// for each contour we found before we loop over them, calculate their area,
		// and then if our area is larger than our minimum area, and our currently found largest area
		// it stores the contour as our largest contour and the area as our largest area
		for (MatOfPoint contour : contours) {
			double area = Imgproc.contourArea(contour);
			if (area > largestContourArea && area > minArea) {
				largestContour = contour;
				largestContourArea = area;
			}
		}
		
		// sets up the center points of our largest contour to be -1 (offscreen)
		largestContourX = largestContourY = -1;
		
		// if we found it, calculates the actual centers
		if (largestContour != null) {
			Moments moment = Imgproc.moments(largestContour);
			largestContourX = (moment.m10 / moment.m00);
			largestContourY = (moment.m01 / moment.m00);
		}
		
		// determines the current prop position, using the left and right dividers we gave earlier
		// if we didn't find any contours which were large enough, sets it to be unfound
		PropPositions propPosition;
		if (largestContour == null) {
			propPosition = PropPositions.UNFOUND;
		} else if (largestContourX < left.getAsDouble()) {
			propPosition = PropPositions.LEFT;
		} else if (largestContourX > right.getAsDouble()) {
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
		if (largestContour != null) {
			Rect rect = Imgproc.boundingRect(largestContour);
			
			float[] points = {rect.x * scaleBmpPxToCanvasPx, rect.y * scaleBmpPxToCanvasPx, (rect.x + rect.width) * scaleBmpPxToCanvasPx, (rect.y + rect.height) * scaleBmpPxToCanvasPx};
			
			canvas.drawLine(points[0], points[1], points[0], points[3], linePaint);
			canvas.drawLine(points[0], points[1], points[2], points[1], linePaint);
			
			canvas.drawLine(points[0], points[3], points[2], points[3], linePaint);
			canvas.drawLine(points[2], points[1], points[2], points[3], linePaint);
			
			String text = String.format(Locale.ENGLISH, "%s", recordedPropPosition.toString());
			
			canvas.drawText(text, (float) largestContourX * scaleBmpPxToCanvasPx, (float) largestContourY * scaleBmpPxToCanvasPx, textPaint);
		}
	}
	
	/**
	 * @return the last found prop position, if none have been found, returns {@link PropPositions#UNFOUND}
	 */
	public PropPositions getRecordedPropPosition() {
		return recordedPropPosition;
	}
	
	// returns the largest contour if you want to get information about it
	public MatOfPoint getLargestContour() {
		return largestContour;
	}
	
	public void close() {
		hierarchy.release();
	}
	
	// the enum that stores the 4 possible prop positions
	public enum PropPositions {
		LEFT,
		MIDDLE,
		RIGHT,
		UNFOUND;
	}
}
