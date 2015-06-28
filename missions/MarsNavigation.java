package missions;

import helperClasses.ArduinoSideSensors;

import java.io.File;
import java.util.HashSet;
import java.util.Set;
import java.util.Stack;

import lejos.hardware.Button;
import lejos.hardware.Sound;
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
	static EV3IRSensor irSeekSensor;
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
	static float[] beaconData = new float[8];
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
	static SensorMode seekSense;
	static int steerPos = 0;
	static Cell turningFrom;
	static Cell prevCell = null;
	static Stack<Cell> cellStack = new Stack<Cell>();
	static final float WALL_SENSITIVITY = 0.15f;
	static Direction currentBearing = Direction.NORTH;
	static Cell finalCell;
	
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
					steeringMotor.rotateTo(steeringRange / 2);
					rightMotor.rotate(backoffLeft, true);
					leftMotor.rotate(backoffLeft);
				}
				endAngle += 90;
				angleSense.fetchSample(gyroAngles, 0);				
				steeringMotor.rotateTo((int) (gyroAngles[0] - endAngle) * 4 );

//				steeringMotor.rotateTo(-(steeringRange/2));
				rightMotor.forward();
				leftMotor.forward();

//				endAngle = correctAngle(gyroAngles[0])+90;
				break;
			case Turning_Right:
				turningFrom = Cell.getCurrentCell();
				int backoffRight = getBackoff(Status.Turning_Right);
				if(backoffRight != 0)
				{
					steeringMotor.rotateTo(-steeringRange / 2);
					rightMotor.rotate(backoffRight, true);
					leftMotor.rotate(backoffRight);
				}

//				steeringMotor.rotateTo(steeringRange/2);
				endAngle -= 90;
				angleSense.fetchSample(gyroAngles, 0);				
				steeringMotor.rotateTo((int) (gyroAngles[0] - endAngle) * 4 );
				
				rightMotor.forward();
				leftMotor.forward();

//				endAngle = correctAngle(gyroAngles[0])-90;
				break;
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
			frontSamples[0] = 0;
			leftMotor.stop();
			rightMotor.stop();
			leftMotor.rotate(-60);
			rightMotor.rotate(-60);
			leftMotor.stop();
			rightMotor.stop();
			correctVeer();
			leftMotor.forward();
			rightMotor.forward();
			break;
		case Turning_Left:
			orig_steer = steeringMotor.getTachoCount();
			leftMotor.stop();
			rightMotor.stop();
//			if( Cell.getCurrentCell() == turningFrom ) //if(gyroAngles[0] <= (endAngle - (4*ANGLE_ERROR_MARGIN)))
//			{
//				System.out.println("Recovering from stall; still in same cell");
//				leftMotor.rotate(-90, true); //Back off 7cm
//				rightMotor.rotate(-90);
//				steeringMotor.rotateTo((steeringRange/3), true);
//				leftMotor.rotate(135, true); //Go forward 7 cm in other direction
//				rightMotor.rotate(135);
//			}
//			else
			{
				System.out.println("Recovering from stall; not in same cell");
				steeringMotor.rotateTo((steeringRange/3), true);
				leftMotor.rotate(-90, true); //Back off 10cm
				rightMotor.rotate(-90);	
			}
			steeringMotor.rotateTo(orig_steer);
			rightMotor.forward();
			leftMotor.forward();
			break;
		case Turning_Right:
			orig_steer = steeringMotor.getTachoCount();
			leftMotor.stop();
			rightMotor.stop();
//			if( Cell.getCurrentCell() == turningFrom )
//			{
//				System.out.println("Recovering from stall; still in same cell");
//				leftMotor.rotate(-90, true); 
//				rightMotor.rotate(-90);
//				steeringMotor.rotateTo(-(steeringRange/3), true);
//				leftMotor.rotate(135, true); 
//				rightMotor.rotate(135);
//			}
//			else
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
		switch(st)
		{
		case Turning_Left:
			if(current.getWallState(currentDir.getLeftDirection()) == Cell.WallState.VIRTUAL_WALL)
				return 0;
			break;
		case Turning_Right:
			if(current.getWallState(currentDir.getRightDirection()) == Cell.WallState.VIRTUAL_WALL)
				return 0;
			break;
		default:
			break;
		}
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
			offset = 0;
		}
		offset *= 0.6;
		int toReturn = (int) ((-offset) * 410);
		if(Math.abs(toReturn) < 20)
			toReturn = -60;
		System.out.println("Backoff: " + toReturn);
		return toReturn;
	}

	public static enum Direction
	{
		NORTH, SOUTH, EAST, WEST;
		
		public static Direction getDirectionFromGyro()
		{
			int modifiedAngle = (int) gyroAngles[0];
			
			//ensure our values are between 0 and 359
			while(modifiedAngle < 0)
				modifiedAngle += 360;
			modifiedAngle %= 360;
			
			if(modifiedAngle >= 45 && modifiedAngle <= 134)
				return WEST;
			else if(modifiedAngle >= 135 && modifiedAngle <= 224)
				return SOUTH;
			else if(modifiedAngle >= 225 && modifiedAngle <= 314)
				return EAST;
			else
				return NORTH;
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
		private int northTimes, southTimes, eastTimes, westTimes;
		
		public int getNorthTimes() {
			return northTimes;
		}

		public int getSouthTimes() {
			return southTimes;
		}

		public int getEastTimes() {
			return eastTimes;
		}

		public int getWestTimes() {
			return westTimes;
		}
		public void clearCounters()
		{
			northTimes = 0;
			southTimes = 0;
			eastTimes = 0;
			westTimes = 0;
		}
		public void incrementCounters(Direction dir)
		{
			if( realX > (0.7*x + 0.20) && realX < (0.7*(x+1) -0.20) &&
				realY > (0.7*y + 0.20) && realY < (0.7*(y+1) -0.20) )
			{
			switch(dir)
			{
			case NORTH:
				northTimes++;
				if(northTimes == 2)
				{
					north = WallState.VIRTUAL_WALL;
					System.out.println("RWALL to VWALL: " + dir + " " + this + " [ " + realX + ", " + realY + "]");
				}
				break;
			case SOUTH:
				southTimes++;
				if(southTimes == 2)
				{
					south = WallState.VIRTUAL_WALL;
					System.out.println("RWALL to VWALL: " + dir + " " + this);
				}
				break;
			case EAST:
				eastTimes++;
				if(eastTimes == 2)
				{
					east = WallState.VIRTUAL_WALL;
					System.out.println("RWALL to VWALL: " + dir + " " + this);
				}
				break;
			case WEST:
				westTimes++;
				if(westTimes == 2)
				{
					west = WallState.VIRTUAL_WALL;
					System.out.println("RWALL to VWALL: " + dir + " " + this);
				}
				break;
			default:
				break;
			}
			}
		}
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
					if(reading <= 410)
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
					if( leftMotor.getTachoCount() > (410 * 6)) // go max speed for 4 meters
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
			leftMotor.setSpeed(130);
			rightMotor.setSpeed(130);

			setStatus(Status.Forward);
			
			startLocationMode();
			Cell current;
				
			for(int i = 1; true; ++i)
			{
			
//Update Sensors
//			tachoReading = leftMotor.getTachoCount(); //TachoCount will get lower when moving backwards
//			System.out.println("After getting values at iteration " + i + ":\t" + (System.nanoTime() - beginningTime));
//			Left Following Strategy
			switch(getStatus()){
			case Backward:
//				leftSense.fetchSample(lSamples, 0);				
//				angleSense.fetchSample(gyroAngles, 0);
//				System.out.println("Current Gyro Angle: " + gyroAngles[0]);

//				System.out.println("After correcting veer at iteration " + i + ":\t" + (System.nanoTime() - beginningTime));

				current = Cell.getCurrentCell();
				if(current != prevCell)
				{
					int cellPosition = cellStack.search(current);
					if(cellPosition != -1)
					{
						for(int n=1;n<cellPosition;n++)
						{
							cellStack.pop();
						}
						current.setWallState(currentBearing, Cell.WallState.VIRTUAL_WALL);
						System.out.println("Right sense: "+bSamples[1]+ "Placing VWALL on"+current.toString()+" For " + currentBearing.toString());
					}
					current.clearCounters();
				}
				prevCell = current;
				bothSense.fetchSample(bSamples, 0);
				if(bSamples[0] < 0.42)
					current.incrementCounters(currentBearing.getLeftDirection());
				if(bSamples[1] < 0.42)
					current.incrementCounters(currentBearing.getRightDirection());
				if(current.getWallState(currentBearing.getRightDirection()) != Cell.WallState.VIRTUAL_WALL 
					&& bSamples[1]>=spaceDist && !(current.equals(turningFrom)))
				{
					setStatus(Status.Turning_Right);
//					System.out.println("End Angle: " + endAngle);
//					angleSense.fetchSample(gyroAngles, 0);
				}
				else if(current.getWallState(currentBearing.getOppositeDirection()) == Cell.WallState.VIRTUAL_WALL || leftMotor.isStalled() || rightMotor.isStalled())
				{
					setStatus(Status.Forward);
				}
				else correctVeer();
				
				System.out.println("Right distance:\t" + bSamples[1]);

				break;
			case Forward:

//				System.out.println("After correcting veer at iteration " + i + ":\t" + (System.nanoTime() - beginningTime));
//				if(beaconData[1]<=15) Was causing null exception
//					setStatus(null);
				current = Cell.getCurrentCell();
				if(current != prevCell)
				{
					int cellPosition = cellStack.search(current);
					if(cellPosition != -1)
					{
						for(int n=1;n<cellPosition;n++)
						{
							cellStack.pop();
						}
						current.setWallState(currentBearing.getOppositeDirection(), Cell.WallState.VIRTUAL_WALL);
					}
					else
					{
						cellStack.push(current);
						prevCell = current;
					}
					current.clearCounters();
				}
				bothSense.fetchSample(bSamples, 0);
//				seekSense.fetchSample(beaconData, 0);

				frontSense.fetchSample(frontSamples, 0);

				if(bSamples[0] < 0.75 * spaceDist)
					current.incrementCounters(currentBearing.getLeftDirection());
				if(bSamples[1] < 0.75 * spaceDist)
					current.incrementCounters(currentBearing.getRightDirection());
				if(leftMotor.isStalled() || rightMotor.isStalled())
					recover(currentStatus);
				if( current != turningFrom ) turningFrom = null;
				
				if(current.getWallState(currentBearing.getLeftDirection()) != Cell.WallState.VIRTUAL_WALL && (bSamples[0]>=spaceDist && !(current.equals(turningFrom))))
				{
					System.out.println("Left sense: " + bSamples[0]);
					setStatus(Status.Turning_Left);
					System.out.println("End Angle: " + endAngle);
					angleSense.fetchSample(gyroAngles, 0);
				}
				else if(current.getWallState(currentBearing) == Cell.WallState.VIRTUAL_WALL || frontSamples[0]<30)
				{
//					System.out.println("Moving Backward " + i);
					setStatus(Status.Backward);
				}
				else correctVeer();
				
				System.out.println("Left distance:\t" + bSamples[0]);
				break;
			case Turning_Left:
				current = Cell.getCurrentCell();
				if(current != prevCell)
				{
					cellStack.push(current);
					prevCell = current;
				}
				if(leftMotor.isStalled() || rightMotor.isStalled() )
				{
					recover(currentStatus);
				}
				else {
				angleSense.fetchSample(gyroAngles, 0);
//				System.out.println("Current Angle: " + gyroAngles[0]);
//				System.out.println("Desired Angle: " + endAngle);
				if(gyroAngles[0] >= (endAngle - (4*ANGLE_ERROR_MARGIN)))
				{
					setStatus(Status.Forward);
//					System.out.println("Moving Forward " + i);
				}
				else
					steeringMotor.rotateTo(-(steeringRange/3), true);
				}
				break;
			case Turning_Right:
				current = Cell.getCurrentCell();
				if(current != prevCell)
				{
					cellStack.push(current);
					prevCell = current;
				}
				if(leftMotor.isStalled() || rightMotor.isStalled() )
				{
					recover(currentStatus);
				}
				else
				{
					angleSense.fetchSample(gyroAngles, 0);
				
//					System.out.println("Current Angle: " + gyroAngles[0]);
//					System.out.println("Desired Angle: " + endAngle);
					if(gyroAngles[0] <= (endAngle + (4 *ANGLE_ERROR_MARGIN)))
					{
						setStatus(Status.Forward);
//						System.out.println("Moving Forward " + i);
					}
					else
						steeringMotor.rotateTo((steeringRange/3), true);
				}
				break;

			default:
				break;
			}
			
			if(finalCell != null && finalCell.equals(Cell.getCurrentCell()))
			{
				leftMotor.stop();
				rightMotor.stop();
				Sound.playSample(new File("tada.wav"), 100);
				for(int i1 = 1; true; i1++)
				{
					Button.LEDPattern(i1 % 2 + 1);
					Delay.msDelay(100);
				}
			}
			
			if(Button.ENTER.isDown() || Button.UP.isDown()) 
			{
				leftMotor.stop(); 
				rightMotor.stop();
				realX = spaceDist / 2.0; //refreshes coordinates
				realY = 0.14; //distance from back of the robot to center
				currentStatus = null;
				turningFrom = null;
				//above commands will cause Cell.getCurrentCell() to return (0, 0)
				cellStack.clear();
				if(Button.ENTER.isDown())
					finalCell = Cell.getCurrentCell();
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
		else if(press == Button.UP.getId() || press == Button.DOWN.getId())
		{
			boolean go_left = (press == Button.DOWN.getId());
			int target_angle = 0;
			leftMotor.setSpeed(200);
			rightMotor.setSpeed(200);

			setStatus(Status.Forward);
			
//			startLocationMode();
				
			for(;;)
			{
				int seek = seekIR(1);
				if(seek>0)
					target_angle = trueMultiplier * seek + 10;
				else if(seek<0)
					target_angle = trueMultiplier * seek - 10;
				bothSense.fetchSample(bSamples, 0);
				frontSense.fetchSample(frontSamples, 0); //TODO Use this as seeker next, keep switching or use different sensor?
	//Update Sensors
//				tachoReading = leftMotor.getTachoCount(); //TachoCount will get lower when moving backwards
//				System.out.println("After getting values at iteration " + i + ":\t" + (System.nanoTime() - beginningTime));
//				Left Following Strategy

//TODO:			target_angle = seekIR(); //Is there a beacon, make it the end angle
				if(seek != 0)
				{
					Button.LEDPattern(4);
					if(frontSamples[0]<=30 || leftMotor.isStalled() || rightMotor.isStalled())
					{
						rightMotor.stop();
						leftMotor.stop();

						if( bSamples[0] < WALL_SENSITIVITY ) {
							go_left = false;
						}
						else if( bSamples[1] < WALL_SENSITIVITY  ) {
							go_left = true;
						}
						
						rightMotor.backward();
						leftMotor.backward();
						Delay.msDelay(500);
						if( go_left )
						{
							steeringMotor.rotateTo(-(steeringRange/4), true); //Go left less, beacon in sight
							
						}
						else
						{
							steeringMotor.rotateTo((steeringRange/4), true); //Go right less, beacon in sight
						}
		
						rightMotor.forward();
						leftMotor.forward();
						Delay.msDelay(1000);
					}
					else //Front clear - pursue goal
					{
						endAngle = target_angle;
					}
				}
				else
				{
					if(frontSamples[0]>30)
					{
						endAngle = target_angle;
					}
					else
					{
						rightMotor.stop();
						leftMotor.stop();

						if( bSamples[0] < WALL_SENSITIVITY ) {
							go_left = false;
						}
						else if( bSamples[1] < WALL_SENSITIVITY  ) {
							go_left = true;
						}
						
						rightMotor.backward();
						leftMotor.backward();
						Delay.msDelay(1000);
						if( go_left )
						{
							steeringMotor.rotateTo(-(steeringRange/3), true); //Start going to left
						}
						else
						{
							steeringMotor.rotateTo((steeringRange/3), true); //Start going to right
						}
						rightMotor.forward();
						leftMotor.forward();
						Delay.msDelay(1000);
					}
				}

				correctVeer(); //steer away from wall/ steer toward endAngle
			}


		}
	}

	private static int seekIR(int channel) {
		int position = 2 * (--channel);
		seekSense.fetchSample(beaconData, 0);
		return (int) beaconData[position];
	}

	private static void correctVeer() {
		int newSteerPos;
		if( bSamples[0] < WALL_SENSITIVITY) //does this for challenge 1
		{
			newSteerPos = (int)((70 * (WALL_SENSITIVITY - bSamples[0])) + 5) * MULTIPLIER; //if close to a wall, veer away from it
			System.out.println( "Too near LEFT : " +newSteerPos);
		}
		else if( bSamples[0] < (0.42) && bSamples[0] > 0.21)
		{
			newSteerPos = (int)(-70 * (bSamples[0] - 0.21) - 5) * MULTIPLIER;
			System.out.println( "Too away LEFT : " +newSteerPos);
		}
		else if( bSamples[1] < WALL_SENSITIVITY)
		{
			newSteerPos = (int)((-70 * (WALL_SENSITIVITY - bSamples[1])) - 5) * MULTIPLIER;
			System.out.println( "Too near RIGHT : " +newSteerPos);
		}
		else if( bSamples[1] < 0.42 && bSamples[1] > 0.21)
		{
			newSteerPos = (int)(70 * (bSamples[1] - 0.21) + 5) * MULTIPLIER;
			System.out.println( "Too away RIGHT : " +newSteerPos);
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
			System.out.println("GYRO based Steering to " + newSteerPos);
		}
	
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
		irSeekSensor = new EV3IRSensor(SensorPort.S3);
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
		seekSense = irSeekSensor.getSeekMode();
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
		xDelta = (avgDelta) * (1.0/410.0) * Math.cos(gyroRadians);
		yDelta = (avgDelta) * (1.0/410.0) * Math.sin(gyroRadians);
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
	
//	private static int correctAngle(float angleEstimate)
//	{
//		float quotient;
//		float remainder;
//		quotient = angleEstimate/90;
//		remainder = angleEstimate%90;
//		if(remainder>=0)
//		{
//			if(remainder>=45)
//			{
//				quotient+=1;
//			}	
//		}
//		else
//		{
//			if(remainder<=-45)
//			{
//				quotient-=1;
//			}
//		}
//		return (int)quotient*90;
//	}
}
