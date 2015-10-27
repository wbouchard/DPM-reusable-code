import lejos.hardware.Sound;
import lejos.robotics.SampleProvider;

import java.util.Arrays;
import java.util.ArrayList;

public class USLocalizer {
	public enum LocalizationType { FALLING_EDGE, RISING_EDGE };
	public static double ROTATION_SPEED = 30;

	private Odometer odo;
	private SampleProvider usSensor;
	private float[] usData;
	private LocalizationType locType;
	
	// add navigation item for easy access to motors and turnTo functions
	private Navigation navi;
	
	// this is used in the median filter to avoid sensor errors
	// every distance collected by the sensor is added to the ArrayList
	public ArrayList<Float> usCollectedData = new ArrayList<Float>();
	
	public USLocalizer(Odometer odo,  SampleProvider usSensor, float[] usData, LocalizationType locType) {
		this.odo = odo;
		this.usSensor = usSensor;
		this.usData = usData;
		this.locType = locType;
		
		this.navi = new Navigation(odo);
	}
	
	public void doLocalization() {
		double [] pos = new double [3];
		double angleA = 0, angleB = 0;
		double deltaAng;
		
		// distance at which we consider that the robot has seen a wall
		// since the width of a tile is 30 cm and the robot starts inside a tile,
		// we define that the robot does not see a wall when distance is > 34
		float d = 34;
		// modifier for the noise margin to allow an error when the robot sees a falling edge
		// or a rising edge
		float k = 2;
		
		// get the first 5 distance points while the robot doesn't move
		for (int i = 0; i < 5; i++)
			getFilteredData();
		
		if (locType == LocalizationType.FALLING_EDGE) {
			// Step 1. rotate the robot until it sees no wall (counter-clockwise, as theta = 0 at positive x-axis)
			// Step 2. keep rotating until the robot sees a wall, then latch the angle
			// Step 3. switch direction and wait until it sees no wall
			// Step 4. keep rotating until the robot sees a wall, then latch the angle
			
			boolean rotateCW = false;
			boolean rotateCounterCW = true;
			boolean noWall = false;
			
			// Step 1
			while (rotateCounterCW) {
				navi.setSpeeds(-100, 100);

				// robot doesn't see a wall
				// want the distance to be greater than d + k for the wall to be considered out of range
				if (getFilteredData() > d + k && !noWall)
					noWall = true;
				
				// Step 2
				// robot had detected prior to this point that there was no wall
				// in front of it. Then, when it sees a wall, it enters this block
				// and saves the angle
				if (getFilteredData() < d - k && noWall) {
					angleA = odo.getAng();
					// move to next step of the routine
					rotateCounterCW = false;
				}	
			}
			
			Sound.beep();
			
			// Step 3
			rotateCW = true;
			noWall = false;
			while (rotateCW) {
				navi.setSpeeds(100, -100);

				if (getFilteredData() > d + k && !noWall)
					noWall = true;

				// Step 4
				if (getFilteredData() < d - k && noWall) {
					angleB = odo.getAng();
					rotateCW = false;
				}
			}
			
			Sound.beep();
			
			// fix angles for wraparound
			if (angleA < 0.0)
				angleA = 360.0 + (angleA % 360.0);
			if (angleB < 0.0)
				angleB = 360.0 + (angleB % 360.0);
			
			// angleA is clockwise from angleB, so assume the average of the
			// angles to the right of angleB is 45 degrees past 'north'
			if (angleA < angleB)
				deltaAng = 45 - (angleA + angleB)/2;
			else
				deltaAng = 225 - (angleA + angleB)/2;
			
			
			// avoid negative angles for turnTo
			double angle = odo.getAng() + deltaAng;
			if (angle < 0)
				angle = 360.0 + angle;
			
			// rotate to desired heading and stop the motors
			pos = odo.getPosition();
			pos[2] = angle;
			odo.setPosition(pos, new boolean [] {true, true, true});
			navi.setSpeeds(0,0);
			
			// turn to positive x axis
			navi.turnTo(180.0,true);
			
			// Update the odometer position. Since the robot has not moved horizontally or vertically, 
			// its (x, y) position should be 0. The robot will face the positive x axis if localization is successful,
			// so theta should be 0*
			pos[2] = 0.0;
			odo.setPosition(pos, new boolean [] {true, true, true});
			
			// turn to face face north
			navi.turnTo(90.0, true);
					
			pos[2] = 90.0;
			odo.setPosition(pos, new boolean [] {true, true, true});	
			
			// sleep before continuing
			try {
				Thread.sleep(3000);
			} catch (Exception e) {
				
			}
		}
	}
	
	private float getFilteredData() {
		usSensor.fetchSample(usData, 0);
		float distance = (float)(usData[0] * 100);
		
		// A stack is used to log distance data, because its size adjusts with the number of values
		usCollectedData.add((Float)distance);
		
		// Median Filter with a window of 5 values
		// used to reject sensor errors, i.e. when distance = 255 unexpectedly
		int filterWindow = 5;
		if (usCollectedData.size() >= filterWindow) {
			// get the last values stored in the ArrayList
			float[] previousData = new float[filterWindow];
			for (int i = 0; i < filterWindow; i++) {
				int index = usCollectedData.size() - 1 - i;
				previousData[i] = (float)usCollectedData.get(index);
			}
			
			Arrays.sort(previousData);
			
			// the array has an odd number of values in it, so the median
			// is the middle value of the array
			distance = previousData[previousData.length / 2];
		}
		
		return distance;
	}
	
	public Navigation getNavi() {
		return this.navi;
	}

}