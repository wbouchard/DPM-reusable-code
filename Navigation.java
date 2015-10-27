package ev3Navigation;

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
//import wallFollower.UltrasonicPoller;

public class Navigation extends Thread {

	// Robot specifications 
	public static final double WB = 15.2; //wheel base in cm
	public static final double WR = 2.1 ; //wheel radius in cm
	private final int motorStraight = 200;
	private final int motorRotate = 100;
	
	// Navigating specifications
	private final double thetaThreshold = 0.05;
	private boolean isNavigating = false;
	private boolean isRotating = false;
	private double[] path = new double[0];
	
	// Current position
	// volatile is required so the Thread doesn't store a cached version of the values
	private volatile double currentX, currentY, currentTheta;
	
	// Angle to rotate to face destination
	public volatile double dTheta; 
	
	public volatile Odometer odometer;
	
	// Set up ultrasonic sensor in Distance mode and get distance from it
	Port port = LocalEV3.get().getPort("S1"); 
	SensorModes sensor = new EV3UltrasonicSensor(port);
	SampleProvider sampleDist = sensor.getMode("Distance");
	public float[] data = new float[sampleDist.sampleSize()];
	
	public PController p;
	public UltrasonicPoller usPoller;
	
	public final int bandCenter = 20; // robot to wall distance 
	public final int bandError = 2; // allowable error threshold
	
	public EV3LargeRegulatedMotor leftMotor, rightMotor;
	
	public Navigation(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, Odometer odometer) {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.odometer = odometer;	
		
		p = new PController(leftMotor, rightMotor, bandCenter, bandError);
		usPoller = new UltrasonicPoller(sampleDist, data, p, odometer);
	}
	
	public void run() {		
		// path order, coordinates are in pairs
		for(int i = 0; i < path.length; i += 2) {
			travelTo(path[i], path[i+1]);
		}
	}
	
	// this method makes the robot travel to the absolute field location x, y
	// by using Odometer information
	public void travelTo(double x, double y) {
		isNavigating = true;
		
		// loop that checks current heading and obstacles
		while(isNavigating) {
			currentX = odometer.getX();
			currentY = odometer.getY();
			currentTheta = odometer.getTheta();
			
			leftMotor.setSpeed(motorStraight);
			rightMotor.setSpeed(motorStraight);
			
			dTheta = calcHeading(currentX, currentY, currentTheta, x, y);

			// Determine which quadrant the destination is in
			// If it is to the right of the robot, turn clockwise, and vice versa
			boolean clockwise;
			/*if ((x - currentX > 0 && y - currentY > 0) || 
				(x - currentX > 0 && y - currentY < 0))*/
			if (x - currentX > 0)
				clockwise = true;
			else
				clockwise = false;
			
			// true if orientation is off from target way-point
			if (dTheta > thetaThreshold)
				turnTo(dTheta, clockwise);
			
			// reset motors after turning
			leftMotor.setSpeed(motorStraight);
			rightMotor.setSpeed(motorStraight);
			leftMotor.forward();
			rightMotor.forward();
			
			// OBSTACLE AVOIDANCE
			usPoller.setStartPos(currentX, currentY, currentTheta);
			// if an obstacle is detected, this method will execute until the robot can continue
			usPoller.obstacleAvoidance();
			
			// Euclidian error used to test whether the robot is within distance of the point.
			currentX = odometer.getX();
			currentY = odometer.getY();
			
			if ((x-currentX)*(x-currentX) + (y-currentY)*(y-currentY) < 1.0) { 
				// the booleans are set so that the motors stop at more or less the same time
				leftMotor.stop(true);
				rightMotor.stop(false);
				Sound.beep();
				
				try {
					Thread.sleep(2000);
				} catch (InterruptedException e) {
					//no error expected here
				}
				
				isNavigating = false;
				return;
			}		
		}
	}
	
	// turn to heading required to get to destination
	public void turnTo(double theta, boolean clockwise) {
		isRotating = true;
		
		// to turn clockwise: left turns forward, right turns backward
		// note that rotate takes degrees as argument
		if (clockwise) {
			leftMotor.setSpeed(motorRotate);
			rightMotor.setSpeed(motorRotate);
			int angle = convertAngle(WR, WB, theta); // degrees
			
			// booleans: don't wait for the left motor to be done rotating
			// before starting the right wheel's rotation
			leftMotor.rotate(angle, true);
			rightMotor.rotate(-angle, false);
		
			isRotating = false;
			return;
		}
		// turn counter-clockwise
		else {
			leftMotor.setSpeed(motorRotate);
			rightMotor.setSpeed(motorRotate);
			int angle = convertAngle(WR, WB, Math.abs(theta));
			leftMotor.rotate(-angle, true);
			rightMotor.rotate(angle, false);
			
			isRotating = false;
			return;
		}
	}	
	
	// returns true if travelTo or turnTo are running in another thread
	public boolean isNavigating() {
		return (this.isNavigating || this.isRotating);
	}
	
	// sets the robot's path
	public void setPath(double[] path) {
		this.path = path;
	}
	
	// convert angle the robot should turn to angle the wheels should turn (radians)
	public static int convertAngle(double WR, double WB, double angle) {
		return (int)Math.toDegrees(angle * (WB/2)/WR);
	}
	
	// calculate distance between two points: d = sqrt( (x1-x0)^2 + (y1-y0)^2 )
	public static double distBetweenTwoPoints(double[] p, double[] q) {
		return Math.sqrt((p[0] - q[0])*(p[0] - q[0]) + ((p[1] - q[1])*(p[1] - q[1])));
	}
	
	// calculate correct heading that the robot has to turn to (radians)
	// theta is 0 when the robot is facing the positive y-axis
	// form a triangle using current point, destination point, and an arbitrary point
	// the arbitrary point is along the robot's current orientation
	// then use the Law of Cosines to find heading.
	// because of the triangle, the angle will never be greater than 180 degrees
	public static double calcHeading(double currentX, double currentY, double currentTheta, double x, double y) {
		double[] p = {currentX, currentY};
		double[] q = {x, y};
		double[] a = {10.0*Math.sin(currentTheta) + currentX, 10.0*Math.cos(currentTheta) + currentY};
		
		double distPQ = distBetweenTwoPoints(p, q); // current - destination
		double distPA = distBetweenTwoPoints(p, a); // vertical side
		double distAQ = distBetweenTwoPoints(a, q); // hypothenuse
		
		double angle = Math.acos((distPQ*distPQ + distPA*distPA - distAQ*distAQ)/(2*distPA*distPQ));
		
		return angle;
	}
}
