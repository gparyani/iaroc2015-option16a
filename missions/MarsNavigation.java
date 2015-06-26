package missions;

import helperClasses.ArduinoSideSensors;

import java.util.HashSet;
import java.util.Set;

import lejos.hardware.Button;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3IRSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class MarsNavigation {

	static EV3LargeRegulatedMotor leftMotor;
	static EV3LargeRegulatedMotor rightMotor;
	static EV3MediumRegulatedMotor steeringMotor;
//	static NXTUltrasonicSensor rightSensor;
//	static NXTUltrasonicSensor leftSensor;
	static ArduinoSideSensors sideSensors;
	static EV3IRSensor irSensor;
	static int steeringRange;
	static final int MULTIPLIER = 3;
	static int trueMultiplier = MULTIPLIER;


	public enum Status{
		Forward, Backward, Turning_Left, Turning_Right,
	}
	
	private static Status currentStatus;
	static int ANGLE_ERROR_MARGIN = 5;
	static float[] frontSamples = new float[1];
	static float[] bSamples = new float[2]; //index 0 is left, 1 is right
//	static float[] lSamples = new float[1];
	static float[] gyroAngles = new float[1];
	static int endAngle = 0;
	static float spaceDist;
	static volatile double realX;
	static volatile double realY;
	static int lPrevTacho = 0;
	static int rPrevTacho = 0;
	static EV3GyroSensor gyro;
	static long beginningTime;
	static SampleProvider bothSense;
//	static SampleProvider leftSense;
	static SampleProvider angleSense;
	static SensorMode frontSense;
	static int steerPos = 0;
	static Cell turningFrom;
	static final float WALL_SENSITIVITY = 0.15f;
	static Direction currentBearing = Direction.NORTH;
	
	public static Status getStatus()
	{
		return currentStatus;
	}
	
	public static void setStatus(Status newStatus)
	{
		if(newStatus != currentStatus)
		{
			System.out.println("State transition: " + newStatus);
			leftMotor.stop();
			rightMotor.stop();
			switch(newStatus){
			case Backward:
				steeringMotor.rotateTo(0);
				rightMotor.backward();
				leftMotor.backward();
				trueMultiplier = -MULTIPLIER;
				break;
			case Forward:
				currentBearing = Direction.getDirectionFromGyro();
				steeringMotor.rotateTo(0);
				rightMotor.forward();
				leftMotor.forward();
				trueMultiplier = MULTIPLIER;
				break;
			case Turning_Left:
				turningFrom = Cell.getCurrentCell();
				int backoffLeft = getBackoff(Status.Turning_Left);
				if(backoffLeft != 0)
				{
					steeringMotor.rotateTo(steeringRange / 3);
					rightMotor.rotate(backoffLeft, true);
					leftMotor.rotate(backoffLeft);
				}

				steeringMotor.rotateTo(-(steeringRange/3));
				rightMotor.forward();
				leftMotor.forward();
				endAngle += 90;
//				endAngle = correctAngle(gyroAngles[0])+90;
				break;
			case Turning_Right:
				turningFrom = Cell.getCurrentCell();
				int backoffRight = getBackoff(Status.Turning_Right);
				if(backoffRight != 0)
				{
					steeringMotor.rotateTo(-steeringRange / 3);
					rightMotor.rotate(backoffRight, true);
					leftMotor.rotate(backoffRight);
				}

				steeringMotor.rotateTo(steeringRange/3);
				rightMotor.forward();
				leftMotor.forward();
				endAngle -= 90;
//				endAngle = correctAngle(gyroAngles[0])-90;
			default:
				break;
			}
		}
		currentStatus = newStatus;
	}
	
	private static void recover(Status st) {
		int orig_steer;
		switch( st )
		{
		case Forward:
			frontSamples[0] = 0 ;
			break;
		case Turning_Left:
			orig_steer = steeringMotor.getTachoCount();
			leftMotor.stop();
			rightMotor.stop();
			if( Cell.getCurrentCell() == turningFrom ) //if(gyroAngles[0] <= (endAngle - (4*ANGLE_ERROR_MARGIN)))
			{
				System.out.println("Recovering from stall; still in same cell");
				leftMotor.rotate(-90, true); //Back off 7cm
				rightMotor.rotate(-90);
				steeringMotor.rotateTo((steeringRange/3), true);
				leftMotor.rotate(135, true); //Go forward 7 cm in other direction
				rightMotor.rotate(135);
			}
			else
			{
				System.out.println("Recovering from stall; not in same cell");
				steeringMotor.rotateTo((steeringRange/3), true);
				leftMotor.rotate(-90, true); //Back off 10cm
				rightMotor.rotate(-90);	
			}
			steeringMotor.rotateTo(orig_steer);
			rightMotor.forward();
			leftMotor.forward();
		case Turning_Right:
			orig_steer = steeringMotor.getTachoCount();
			leftMotor.stop();
			rightMotor.stop();
			if( Cell.getCurrentCell() == turningFrom )
			{
				System.out.println("Recovering from stall; still in same cell");
				leftMotor.rotate(-90, true); 
				rightMotor.rotate(-90);
				steeringMotor.rotateTo(-(steeringRange/3), true);
				leftMotor.rotate(135, true); 
				rightMotor.rotate(135);
			}
			else
			{
				System.out.println("Recovering from stall; not in same cell");
				steeringMotor.rotateTo(-(steeringRange/3), true);
				leftMotor.rotate(-90, true); 
				rightMotor.rotate(-90);	
			}
			steeringMotor.rotateTo(orig_steer);
			rightMotor.forward();
			leftMotor.forward();
			break;
		case Backward:
			break;
			
		default: 
			break;
			
		}
	}

	private static int getBackoff(Status st) {
		double offset = 0;
		Cell current = Cell.getCurrentCell();
		Direction currentDir = currentBearing;
		System.out.println(currentDir);
		System.out.println("[" + realX + ", " + realY + "]");
		switch(currentDir)
		{
		case EAST:
			offset = realX - (current.getX() * spaceDist);
			break;
		case NORTH:
			offset = realY - (current.getY() * spaceDist);
			break;
		case SOUTH:
			offset = ((current.getY() + 1) * spaceDist) - realY;
			break;
		case WEST:
			offset = ((current.getX() + 1) * spaceDist) - realX;
			break;
		default:
			return -60;
		}
		if(currentStatus == Status.Backward)
		{
			offset = spaceDist - offset;
		}
		if(offset > 0.25)
			offset -= 0.20;
//		if(st == Status.Turning_Left) 
//		{
//			if(offset > 0.10)
//				offset -= 0.05;
//		}
//		if (st == Status.Turning_Right)
//		{
//			if(offset < 0.60)
//				offset += 0.05;
//		}
		int toReturn = (int) ((-offset) * 412);
		if(Math.abs(toReturn) < 30)
			toReturn = 0;
		System.out.println("Backoff: " + toReturn);
		return toReturn;
	}

	public static enum Direction
	{
		NORTH, SOUTH, EAST, WEST, IN_BETWEEN;
		
		public static Direction getDirectionFromGyro()
		{
			int modifiedAngle = (int) gyroAngles[0];
			
			//ensure our values are between 0 and 359
			while(modifiedAngle < 0)
				modifiedAngle += 360;
			modifiedAngle %= 360;
			
			if(modifiedAngle >= 90 - 4*ANGLE_ERROR_MARGIN && modifiedAngle <= 90 + 4*ANGLE_ERROR_MARGIN)
				return WEST;
			else if(modifiedAngle >= 180 - 4*ANGLE_ERROR_MARGIN && modifiedAngle <= 180 + 4*ANGLE_ERROR_MARGIN)
				return SOUTH;
			else if(modifiedAngle >= 270 - 4*ANGLE_ERROR_MARGIN && modifiedAngle <= 270 + 4*ANGLE_ERROR_MARGIN)
				return EAST;
			else if(modifiedAngle >= 360 - 4*ANGLE_ERROR_MARGIN || modifiedAngle <= 4*ANGLE_ERROR_MARGIN)
				return NORTH;
			
			return IN_BETWEEN;
		}
		
		public Direction getOppositeDirection()
		{
			switch(this) {
			case NORTH:
				return SOUTH;
			case EAST:
				return WEST;
			case SOUTH:
				return NORTH;
			case WEST:
				return EAST;
			default:
				break;
			}
			
			return this;    //IN_BETWEEN's opposite direction is also IN_BETWEEN; compiler throws error if in switch statement itself

		}
		
		Direction getLeftDirection()
		{
			switch(this) {
			case NORTH:
				return WEST;
			case EAST:
				return NORTH;
			case SOUTH:
				return EAST;
			case WEST:
				return SOUTH;
			default:
				break;
			}
			return this;
		}
		
		Direction getRightDirection()
		{
			switch(this) {
			case NORTH:
				return EAST;
				case EAST:
					return SOUTH;
				case SOUTH:
					return WEST;
				case WEST:
					return NORTH;
				default:
					break;
                }
                return this;
        }
	}
	
	public static class Cell
	{
		static Set<Cell> cells = new HashSet<Cell>();
		
		public enum WallState
		{
			UNKNOWN, NO_WALL, REAL_WALL, VIRTUAL_WALL
		}
		
		private int x, y;
		private WallState north = WallState.UNKNOWN,
				south = WallState.UNKNOWN,
				east = WallState.UNKNOWN,
				west = WallState.UNKNOWN;
		
		public int getX() {
			return x;
		}

		public int getY() {
			return y;
		}
		
		public void setWallState(Direction dir, WallState state)
		{
			switch(dir)
			{
			case NORTH:
				north = state;
				break;
			case SOUTH:
				south = state;
				break;
			case WEST:
				west = state;
				break;
			case EAST:
				east = state;
				break;
			default:
				break;
			}
		}
		
		public WallState getWallState(Direction dir)
		{
			switch(dir)
			{
			case NORTH:
				return north;
			case SOUTH:
				return south;
			case WEST:
				return west;
			case EAST:
				return east;
			default:
				break;
			}
			throw new IllegalArgumentException("Can't obtain wall of IN_BETWEEN border");
		}
		
		public boolean equals(Object obj)
		{
			if(obj instanceof Cell)
			{
				Cell cell = (Cell) obj;
				return (cell.x == x) && (cell.y == y);
			}
			else
				return false;
		}
		
		@Override
		public int hashCode()
		{
			return x * 3737 + y;
		}

		private Cell(int x, int y)
		{
			this.x = x;
			this.y = y;
		}
		
		@Override
		public String toString()
		{
			return "Cell (" + x + ", " + y + ")";
		}
		
		public static Cell getCurrentCell()
		{
			double y = realY;
			int yCoord = (int)(y / spaceDist);
			double x = realX;
			int xCoord = (int)(x / spaceDist);
			if(realX < 0)
				xCoord -= 1;	//Adjust for integer division
			
			Cell cache = new Cell(xCoord, yCoord);
			
			if(cells.contains(cache))
			{
				for(Cell cell: cells)
				{
					if(cell.equals(cache))
						return cell;
				}
			}
			else
			{
				cells.add(cache);
				return cache;
			}
			
			return null;	//will never reach here
		}
	}
	
	public static void alignSteering()
	{
		steeringMotor.forward();
		
		while(!steeringMotor.isStalled());
		
		steeringMotor.stop();
		Delay.msDelay(250);
		int rCount = steeringMotor.getTachoCount();
		System.out.println(rCount);
		
		steeringMotor.backward();
		
		while(!steeringMotor.isStalled());
		
		steeringMotor.stop();
		Delay.msDelay(250);
		int lCount = steeringMotor.getTachoCount();
		System.out.println(lCount);
		
		steeringRange = Math.abs(rCount-lCount);
		int average = (lCount + rCount)/2;
		System.out.println(average);
		
		steeringMotor.rotateTo(average);
		steeringMotor.resetTachoCount();
	
	}
	
	public static void main(String[] args) {
		setExitMode();
		
		initializeMotors();
		
		initializeSensors();
		
		alignSteering();
		
		Button.LEDPattern(2);
		calibrateGyro();
			
		beginningTime = System.nanoTime();
		
//Loop Function
		Button.LEDPattern(1);
		System.out.println("Calibration complete");
		
		int press = Button.waitForAnyPress();
		
		if(press == Button.LEFT.getId()) //run challenge 1
		{
			float speed = (int) rightMotor.getMaxSpeed();
			leftMotor.setSpeed(speed);
			rightMotor.setSpeed(speed);
			setStatus(Status.Forward);
			while(true)
			{
				int reading = rightMotor.getTachoCount();
				
				switch(getStatus()){
				case Backward:
					correctVeer();
					if(reading <= 412)
					{
						speed = rightMotor.getMaxSpeed() / 4;
						rightMotor.setSpeed(speed);
						leftMotor.setSpeed(speed);
						rightMotor.backward();
						leftMotor.backward();
					}
					if(rightMotor.isStalled() || leftMotor.isStalled())
					{
						rightMotor.stop();
						leftMotor.stop();
					}
					break;
				case Forward:
					frontSense.fetchSample(frontSamples, 0);
					bothSense.fetchSample(bSamples, 0);
//					rightSense.fetchSample(rSamples, 0);
					System.out.println("Front Reading: " + frontSamples[0]);
					correctVeer();
					if( leftMotor.getTachoCount() > (412 * 6)) // go max speed for 4 meters
					{
						if(frontSamples[0] <= 40) // if it is within 40 cm
						{
							if(speed != rightMotor.getMaxSpeed() / 6) //and speed has not been reduced
							{
								System.out.println("Reducing speed to 1/6"); 
//								rightMotor.flt();
//								leftMotor.flt();
								speed = rightMotor.getMaxSpeed() / 6; //reduce speed to 1/6
								rightMotor.setSpeed(speed);
								leftMotor.setSpeed(speed);
								rightMotor.forward();
								leftMotor.forward();
							}
						}
						else //if it has gone 6 meters but not within 40 cm of wall
						{
							if(speed != rightMotor.getMaxSpeed() / 2) //and speed has not been reduced
							{
								System.out.println("Reducing speed to 1/2");
//								rightMotor.flt();
//								leftMotor.flt();
								speed = rightMotor.getMaxSpeed() / 2; //reduce speed to 1/2
								rightMotor.setSpeed(speed);
								leftMotor.setSpeed(speed);
								rightMotor.forward();
								leftMotor.forward();
							}
						}
					}
					if(rightMotor.isStalled() || frontSamples[0] <= 20) //if motors are stalled then go backwards
					{
						rightMotor.setSpeed(rightMotor.getMaxSpeed());
						leftMotor.setSpeed(leftMotor.getMaxSpeed());
						rightMotor.stop();
						leftMotor.stop();
						setStatus(Status.Backward);
					}
					break;
				case Turning_Left:
					break;
				case Turning_Right:
					break;
				default:
					break;
				}
			}
		}		
		else if(press == Button.RIGHT.getId())
		{
			leftMotor.setSpeed(125);
			rightMotor.setSpeed(125);

			setStatus(Status.Forward);
			
			startLocationMode();
				
			for(int i = 1; true; ++i)
			{
			
//Update Sensors
//			tachoReading = leftMotor.getTachoCount(); //TachoCount will get lower when moving backwards
//			System.out.println("After getting values at iteration " + i + ":\t" + (System.nanoTime() - beginningTime));
//			Left Following Strategy
			switch(getStatus()){
			case Backward:
				bothSense.fetchSample(bSamples, 0);
				System.out.println("Right distance:\t" + bSamples[1]);
//				leftSense.fetchSample(lSamples, 0);				
//				angleSense.fetchSample(gyroAngles, 0);
//				System.out.println("Current Gyro Angle: " + gyroAngles[0]);
				correctVeer();
//				System.out.println("After correcting veer at iteration " + i + ":\t" + (System.nanoTime() - beginningTime));

				Cell rCurrent = Cell.getCurrentCell();
				if(rCurrent.getWallState(currentBearing.getRightDirection()) != Cell.WallState.VIRTUAL_WALL 
					&& bSamples[1]>=spaceDist && !(rCurrent.equals(turningFrom)))
				{
					System.out.println("Right sense: "+bSamples[1]+ "Placing VWALL on"+rCurrent.toString()+" For " + currentBearing.toString());
					rCurrent.setWallState(currentBearing, Cell.WallState.VIRTUAL_WALL);
					setStatus(Status.Turning_Right);
//					System.out.println("End Angle: " + endAngle);
					angleSense.fetchSample(gyroAngles, 0);
					
				}
				else if(rCurrent.getWallState(currentBearing.getOppositeDirection()) == Cell.WallState.VIRTUAL_WALL || leftMotor.isStalled() || rightMotor.isStalled())
				{
					setStatus(Status.Forward);
				}
				break;
			case Forward:
				bothSense.fetchSample(bSamples, 0);
				
				correctVeer();
				frontSense.fetchSample(frontSamples, 0);
				System.out.println("Left distance:\t" + bSamples[0]);
//				System.out.println("After correcting veer at iteration " + i + ":\t" + (System.nanoTime() - beginningTime));
				Cell lCurrent = Cell.getCurrentCell();
				if( lCurrent != turningFrom ) turningFrom = null;
				
				if(lCurrent.getWallState(currentBearing.getLeftDirection()) != Cell.WallState.VIRTUAL_WALL && (bSamples[0]>=spaceDist && !(lCurrent.equals(turningFrom))))
				{
					System.out.println("Left sense: " + bSamples[0]);
					setStatus(Status.Turning_Left);
					System.out.println("End Angle: " + endAngle);
					angleSense.fetchSample(gyroAngles, 0);
				}
				else if(lCurrent.getWallState(currentBearing) == Cell.WallState.VIRTUAL_WALL || frontSamples[0]<30)
				{
//					System.out.println("Moving Backward " + i);
					setStatus(Status.Backward);
				}
				break;
			case Turning_Left:
				if(leftMotor.isStalled() || rightMotor.isStalled() )
				{
					recover(currentStatus);
				}
				angleSense.fetchSample(gyroAngles, 0);
//				System.out.println("Current Angle: " + gyroAngles[0]);
//				System.out.println("Desired Angle: " + endAngle);
				if(gyroAngles[0] >= (endAngle - (4*ANGLE_ERROR_MARGIN)))
				{
					setStatus(Status.Forward);
//					System.out.println("Moving Forward " + i);
				}
				break;
			case Turning_Right:
				if(leftMotor.isStalled() || rightMotor.isStalled() )
				{
					recover(currentStatus);
				}
				angleSense.fetchSample(gyroAngles, 0);
//				System.out.println("Current Angle: " + gyroAngles[0]);
//				System.out.println("Desired Angle: " + endAngle);
				if(gyroAngles[0] <= (endAngle + (4 *ANGLE_ERROR_MARGIN)))
				{
					setStatus(Status.Forward);
//					System.out.println("Moving Forward " + i);
				}
				break;
			default:
				break;
			}
			
			if(Button.ENTER.isDown()) 
			{
				leftMotor.stop(); 
				rightMotor.stop();
				realX = spaceDist / 2.0; //refreshes coordinates
				realY = 0.14; //distance from back of the robot to center
				currentStatus = null;
				turningFrom = null;
				//above commands will cause Cell.getCurrentCell() to return (0, 0)
				Button.LEDPattern(4);
				Button.waitForAnyPress();
				Button.LEDPattern(1);
				gyroAngles[0] = 0;
				Delay.msDelay(200);
				gyro.reset();
				Delay.msDelay(2000);
				leftMotor.resetTachoCount();
				rightMotor.resetTachoCount();
				endAngle = 0;
				currentBearing = Direction.NORTH;
				setStatus(Status.Forward);
			}
			
			System.out.println(Cell.getCurrentCell());
			}
		
		}	
		
	}

	private static void correctVeer() {
		int newSteerPos;
		if( bSamples[0] < WALL_SENSITIVITY) //does this for challenge 1
		{
			newSteerPos = (int)((60 * (WALL_SENSITIVITY - bSamples[0])) + 4) * MULTIPLIER; //if close to a wall, veer away from it
		}
		else if( bSamples[1] < WALL_SENSITIVITY)
		{
			newSteerPos = (int)((-60 * (WALL_SENSITIVITY - bSamples[1])) - 4) * MULTIPLIER;
		}
		else
		{
			angleSense.fetchSample(gyroAngles, 0);
			if(gyroAngles[0] < (endAngle - ANGLE_ERROR_MARGIN) || gyroAngles[0] > (endAngle + ANGLE_ERROR_MARGIN))
				newSteerPos = (int) (gyroAngles[0] - endAngle);
			else
				newSteerPos = 0;
//			newSteerPos += (int)Math.signum(newSteerPos) * 4;
			newSteerPos *= trueMultiplier;
		}
		

		
		System.out.println("Steering to " + steerPos);
		
		if( steerPos != newSteerPos )
		{
			steeringMotor.rotateTo( newSteerPos );
			steerPos = newSteerPos;
		}
		
	}

	private static void setExitMode() {
		new Thread(new Runnable() {
			public void run()
			{
				while(Button.ESCAPE.isUp());
				System.exit(0);
			}
		}).start();
	}

	private static void calibrateGyro() {
		System.out.println("Calibrating gyro...ROBOT MUST BE MOTIONLESS");
		Delay.msDelay(200);
		gyro = new EV3GyroSensor(SensorPort.S1);
		Delay.msDelay(2000);
		System.out.println("Calibration complete!");
		angleSense = gyro.getAngleMode();
	}

	private static void initializeSensors() {
//		rightSensor = new NXTUltrasonicSensor(SensorPort.S3);
//		leftSensor = new NXTUltrasonicSensor(SensorPort.S2);
		sideSensors = new ArduinoSideSensors(SensorPort.S2, 4);
		irSensor = new EV3IRSensor(SensorPort.S4);

		bothSense = sideSensors.getBothSensorMode();
		//Adapts to all size boxes
		bothSense.fetchSample(bSamples, 0);
		float leftDist = bSamples[0];
//		rightSense.fetchSample(bSamples, 0);
		float rightDist = bSamples[1];
//		spaceDist = leftDist + rightDist + 0.185f+0.0762f; //19=robot + 0.0762 = wall width
		spaceDist = 0.7f;
		realX = spaceDist / 2.0;
		realY = 0.14;
		System.out.println("Left Space: " + leftDist);
		System.out.println("Right Space: " + rightDist);
		System.out.println("Total Space:" + spaceDist);
		frontSense = irSensor.getDistanceMode();
//		rightSensor.enable();
//		leftSensor.enable();
//		rightSense = rightSensor.getDistanceMode();
//		leftSense = leftSensor.getDistanceMode();
	}

	private static void initializeMotors() {
		leftMotor = new EV3LargeRegulatedMotor(MotorPort.B);
		rightMotor = new EV3LargeRegulatedMotor(MotorPort.C);
		steeringMotor = new EV3MediumRegulatedMotor(MotorPort.A);

		steeringMotor.setSpeed(steeringMotor.getMaxSpeed());
	}
	private static void updateLocation() {
		int lTachoDelta, rTachoDelta;
		double avgDelta;
		double xDelta, yDelta;
		double gyroRadians;
		lTachoDelta = leftMotor.getTachoCount() - lPrevTacho;
		rTachoDelta = rightMotor.getTachoCount() - rPrevTacho;
		lPrevTacho += lTachoDelta;
		rPrevTacho += rTachoDelta;
		angleSense.fetchSample(gyroAngles, 0);
		gyroRadians = Math.toRadians(gyroAngles[0] + 90);
		avgDelta = (lTachoDelta+rTachoDelta)/2.0;
		xDelta = (avgDelta) * (1.0/412.0) * Math.cos(gyroRadians);
		yDelta = (avgDelta) * (1.0/412.0) * Math.sin(gyroRadians);
		realX += xDelta;
		realY += yDelta;
	}
	private static void startLocationMode() {
		Thread t = new Thread(new Runnable()  {
			public void run()
			{
				while( true )
				{
					updateLocation();
//					System.out.println(getStatus().toString()+", x: "+realX + ", y: " +realY + " left: " + lSamples[0] + " right: " + rSamples[0] );
				}
			}
		});
//		t.setPriority(Thread.MAX_PRIORITY);
		t.start();
	}
	
	private static int correctAngle(float angleEstimate)
	{
		float quotient;
		float remainder;
		quotient = angleEstimate/90;
		remainder = angleEstimate%90;
		if(remainder>=0)
		{
			if(remainder>=45)
			{
				quotient+=1;
			}	
		}
		else
		{
			if(remainder<=-45)
			{
				quotient-=1;
			}
		}
		return (int)quotient*90;
	}
}
