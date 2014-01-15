/**
 * 18649 Fall 2013
 * Team 18
 * Yi Huo (yhuo)
 * Mustafa Bilgen (mbilgen)
 * Sairam Krishnan (sbkrishn)
 * Abhijit Lele (alele)
 */
package simulator.elevatorcontrol;

import simulator.elevatormodules.*;
import simulator.framework.*;
import simulator.payloads.AtFloorPayload.ReadableAtFloorPayload;
import simulator.payloads.CarCallPayload.ReadableCarCallPayload;
import simulator.payloads.CarLanternPayload.ReadableCarLanternPayload;
import simulator.payloads.DrivePayload.ReadableDrivePayload;
import simulator.payloads.DoorClosedPayload.ReadableDoorClosedPayload;
import simulator.payloads.DoorMotorPayload.ReadableDoorMotorPayload;
import simulator.payloads.DriveSpeedPayload.ReadableDriveSpeedPayload;
import simulator.payloads.HallCallPayload.ReadableHallCallPayload;
import simulator.payloads.HallLightPayload.ReadableHallLightPayload;
import simulator.payloads.CarLightPayload.ReadableCarLightPayload;
import simulator.payloads.*;

/**
 * This example monitor shows how to use the RuntimeMonitor hooks to check for
 * fast speed between floors. You will need to implement additional checks
 * to fulfill the project 11 requirements.
 * 
 * See the documentation of simulator.framework.RuntimeMonitor for more details.
 * 
 * @author Justin Ray
 */
public class Proj11RuntimeMonitor extends RuntimeMonitor
{

	protected int currentFloor = MessageDictionary.NONE;
	protected int lastStoppedFloor = MessageDictionary.NONE;
	protected int numReversals = 0;
	protected boolean fastSpeedReached = false;
	protected boolean lanternUp = false;
	protected boolean lanternDn = false;
	protected boolean DoorClosed = true;
	Direction ShownDir = Direction.STOP;
	private final int NO_FLOOR = -1;
	private Speed prevSpeed = Speed.STOP;
	private DoorCommand previousCommand = DoorCommand.STOP;
	private DoorStateMachine doorState = new DoorStateMachine();

	private int r6Violations = 0;
	private int r7Violations = 0;
	private int r8_1Violations = 0;
	private int r8_2Violations = 0;
	private int r8_3Violations = 0;
	private int r9Violations = 0;
	private int r10Violations = 0;

	private boolean[][] atFloorArray = new boolean[Elevator.numFloors][2];


	public Proj11RuntimeMonitor()
	{
		//initialization goes here
	}

	private int getCurrentFloor()
	{
		for (int i = 0; i < Elevator.numFloors; i++)
		{
			if (atFloorArray[i][0] || atFloorArray[i][1])
				return i + 1;
		}
		return NO_FLOOR;
	}

	public void timerExpired(Object callbackData)
	{
		//implement time-sensitive behaviors here
	}

	@Override
	public void receive(ReadableAtFloorPayload msg)
	{
		atFloorArray[msg.getFloor() - 1][msg.getHallway().ordinal()] = msg.getValue();
		updateCurrentFloor(msg);
	}

	@Override
	public void receive(ReadableDrivePayload msg)
	{
		checkFastSpeed(msg);
		checkStopWithoutCalls(msg);

	}

	@Override
	public void receive(ReadableDoorClosedPayload msg)
	{
		
		checkLanternUsage(msg);
		if (msg.isClosed() == true)
		{
			DoorClosed = true;
		}
		else
		{
			DoorClosed = false;
		}
		checkDoorOpenWithoutCalls(msg);
	}

	@Override
	public void receive(ReadableDoorMotorPayload msg)
	{
		checkNudgeConditions(msg);
	}

	@Override
	public void receive(ReadableCarLanternPayload msg)
	{

		checkLanternChange(msg);
		if (DoorClosed == false)
		{
			checkCarMoveDir(msg);
		}

	}

	/**
	 * Warn if stopping at a floor with out a valid hall or car call.
	 * 
	 * @param msg
	 */
	private void checkStopWithoutCalls(ReadableDrivePayload msg)
	{
		int floor = getCurrentFloor();
		Speed speed = msg.speed();

		// commanded to stop when car is moving
		if (speed == Speed.STOP && floor != NO_FLOOR && speed != prevSpeed
				&& lastStoppedFloor != floor && checkHallCall(floor - 1) == false
				&& checkCarCall(floor - 1) == false)
		{
			warning("R-T.6 Violated: Stopped at floor " + floor
					+ " without a hall call or car call!");
			r6Violations++;
		}

		// update previous speed
		prevSpeed = speed;
	}

	/**
	 * Return true if there's a hall call at floor f
	 * 
	 * @param f
	 */
	private boolean checkHallCall(int f)
	{
		return (hallLights[f][0][0].lighted() || hallLights[f][0][1].lighted()
				|| hallLights[f][1][0].lighted() || hallLights[f][1][1].lighted());
	}

	/**
	 * Return true if there's a car call at floor f
	 * 
	 * @param f
	 */
	private boolean checkCarCall(int f)
	{
		return (carLights[f][0].lighted() || carLights[f][1].lighted());
	}

	/**
	 * Warn if doors open without a valid hall or car call.
	 * 
	 * @param msg
	 */
	private void checkDoorOpenWithoutCalls(ReadableDoorClosedPayload msg)
	{
		int floor = getCurrentFloor();
		// doors opened
		if (msg.isClosed() == false)
		{
			// check to see if there were any valid hall or car calls
			if (checkCarCall(floor - 1) == false && checkHallCall(floor - 1) == false &&
				(doorState.getDoorStateFront() != DoorState.REVERSED || doorState.getDoorStateBack() != DoorState.REVERSED) && (floor != lastStoppedFloor))
			{
				warning("R-T.7 Violate: Door opening at floor " + getCurrentFloor()
						+ " without a hall or car call!");
				r7Violations++;
			}
		}
	}

	/**
	 * Warn if both car lanterns are on at the same time when the elevator doors
	 * open.
	 * 
	 * @param msg
	 */
	private void checkLanternUsage(ReadableDoorClosedPayload msg)
	{
		int floor = getCurrentFloor();

		//If the doors are not closed, and there are valid hall or car calls:
		if (msg.isClosed() == false
				&& (checkCarCall(floor - 1) == true || checkHallCall(floor - 1) == false)
				&& lanternUp == true && lanternDn == true)
		{
			warning("R-T.8.1 Violated : Both car lanterns are on at the same time.");
			r8_1Violations++;
		}
	}


	/**
	 * Warn if the CarLantern changes state when doors are open
	 * 
	 * @param msg
	 */
	private void checkLanternChange(ReadableCarLanternPayload msg)
	{
		if ((msg.getDirection() == Direction.UP) && (DoorClosed == false)) //Check the Car Lantern Message for 'Up' message
		{
			if (!msg.lighted() && lanternUp == true)
			{
				lanternUp = false; // turn off the Up lantern if the CarLantern message denotes 'not lighted'
			}
			else if (msg.lighted() && lanternUp == false) // turn on the lantern in the Up direction
			{
				if (ShownDir == Direction.STOP) // set the shown direction to Up
					ShownDir = Direction.UP;
				if (Direction.UP != ShownDir) //executed if the shown direction is not equal to 'Up'
				{
					warning("R-T8.2 Violated: Lantern direction changed!");
					r8_2Violations++;
				}
				lanternUp = true; // Make the Up lantern in 'On' state
			}
		}
		else if ((msg.getDirection() == Direction.DOWN) && (DoorClosed == false))
		{
			if (!msg.lighted() && lanternDn == true) // Turn off Down-Lantern if the  message denotes 'not lighted'
			{
				lanternDn = false;
			}
			else if (msg.lighted() && lanternDn == false)// turn on the lantern in the Down direction
			{
				if (ShownDir == Direction.STOP)// set the shown direction to Down 			
					ShownDir = Direction.DOWN;
				if (Direction.DOWN != ShownDir)
				{
					warning("R-T8.2 Violated: Lantern direction changed!");
					r8_2Violations++;
				}
				lanternDn = true; // Make the down lantern in 'On' state
			}
		}
		else if (DoorClosed == true)
		{
			// Do Nothing since Doors may be open!
			ShownDir = Direction.STOP;
		}
	}

	/**
	 * Warn if Elevator moves in opposite direction of the CarLanterns
	 * 
	 * @param msg
	 */
	private void checkCarMoveDir(ReadableCarLanternPayload msg)
	{
		int desired_floor = mDesiredFloor.getFloor();
		int current_floor = getCurrentFloor();
		if ((current_floor < desired_floor) && (ShownDir != Direction.STOP))
		{
			if (ShownDir != Direction.UP)
			{
				warning("R-T8.3 Violated: car is moving in direction " + Direction.UP
						+ " while actual displayed direction is " + ShownDir);
				r8_3Violations++;
			}
		}
		else if ((current_floor > desired_floor) && (ShownDir != Direction.STOP))
		{
			if (ShownDir != Direction.DOWN)
			{
				warning("R-T8.3 Violated: car is moving in direction " + Direction.DOWN
						+ " while actual displayed direction is " + ShownDir);
				r8_3Violations++;
			}
		}

	}

	/**
	 * Warn if the drive was never commanded to fast when fast speed could be
	 * used.
	 * 
	 * @param msg
	 */
	private void checkFastSpeed(ReadableDrivePayload msg)
	{
		if (msg.speed() == Speed.STOP && currentFloor != MessageDictionary.NONE)
		{
			//stopped at a floor
			if (lastStoppedFloor != currentFloor)
			{
				//we've stopped at a new floor
				if (fastSpeedAttainable(lastStoppedFloor, currentFloor))
				{
					//check and see if the drive was ever reached fast
					if (!fastSpeedReached)
					{
						warning("R-T.9 Violated: The drive was not commanded to FAST on the trip between "
								+ lastStoppedFloor + " and " + currentFloor);
						r9Violations++;
					}
				}
				//now that the check is done, set the lastStoppedFloor to this floor
				lastStoppedFloor = currentFloor;
				//reset fastSpeedReached
				fastSpeedReached = false;
			}
		}
		if (msg.speed() == Speed.FAST)
		{
			//if the drive exceeds the Slow Speed, the drive must have been commanded to fast speed.
			fastSpeedReached = true;
		}
	}

	/**
	 * Warn if the DoorMotor was commanded to Nudge before any door reversals
	 * have occurred.
	 */
	private void checkNudgeConditions(ReadableDoorMotorPayload msg)
	{
		DoorCommand currentCommand = msg.command();
		if (currentCommand == DoorCommand.OPEN && previousCommand == DoorCommand.CLOSE) {
			numReversals++;
		}
		if (currentCommand == DoorCommand.NUDGE && numReversals == 0) {
			warning("R-T.10 Violated: For the first door reversal, the door motor was commanded to NUDGE instead of OPEN.");
			r10Violations++;
		}
		previousCommand = currentCommand;
	}

	/*--------------------------------------------------------------------------
	 * Utility and helper functions
	 *------------------------------------------------------------------------*/
	/**
	 * Computes whether fast speed is attainable. In general, it is attainable
	 * between any two floors.
	 * 
	 * @param startFloor
	 * @param endFloor
	 * @return true if Fast speed can be commanded between the given floors,
	 * otherwise false
	 */
	private boolean fastSpeedAttainable(int startFloor, int endFloor)
	{
		//fast speed is attainable between all floors
		if (startFloor == MessageDictionary.NONE || endFloor == MessageDictionary.NONE)
		{
			return false;
		}
		if (startFloor != endFloor)
		{
			return true;
		}
		return false;
	}


	private void updateCurrentFloor(ReadableAtFloorPayload lastAtFloor)
	{
		if (lastAtFloor.getFloor() == currentFloor)
		{
			//the atFloor message is for the currentfloor, so check both sides to see if they a
			if (!atFloors[lastAtFloor.getFloor() - 1][Hallway.BACK.ordinal()].value()
					&& !atFloors[lastAtFloor.getFloor() - 1][Hallway.FRONT.ordinal()].value())
			{
				//both sides are false, so set to NONE
				currentFloor = MessageDictionary.NONE;
			}
			//otherwise at least one side is true, so leave the current floor as is
		}
		else
		{
			if (lastAtFloor.value())
			{
				currentFloor = lastAtFloor.getFloor();
			}
		}
	}


	private static enum DoorState
	{
		CLOSED, REVERSED, CLOSING, NUDGE
	}

	private class DoorStateMachine
	{

		DoorState[] state = new DoorState[2];


		public DoorStateMachine()
		{
			state[Hallway.FRONT.ordinal()] = DoorState.CLOSED;
			state[Hallway.BACK.ordinal()] = DoorState.CLOSED;
		}

		public DoorState getDoorStateFront() {
			return state[Hallway.FRONT.ordinal()];
		}

		public DoorState getDoorStateBack() {
			return state[Hallway.BACK.ordinal()];
		}

		public void receive(ReadableDoorMotorPayload msg)
		{
			updateState(msg.getHallway());
		}

		public boolean allDoorMotorsStopped(Hallway h)
		{
			return doorMotors[h.ordinal()][Side.LEFT.ordinal()].command() == DoorCommand.STOP
					&& doorMotors[h.ordinal()][Side.RIGHT.ordinal()].command() == DoorCommand.STOP;
		}

		public boolean allDoorMotorsClosing(Hallway h)
		{
			return doorMotors[h.ordinal()][Side.LEFT.ordinal()].command() == DoorCommand.CLOSE
					&& doorMotors[h.ordinal()][Side.RIGHT.ordinal()].command() == DoorCommand.CLOSE;
		}

		public boolean allDoorMotorsOpening(Hallway h)
		{
			return doorMotors[h.ordinal()][Side.LEFT.ordinal()].command() == DoorCommand.OPEN
					&& doorMotors[h.ordinal()][Side.RIGHT.ordinal()].command() == DoorCommand.OPEN;
		}

		public boolean allDoorMotorsNudge(Hallway h)
		{
			return doorMotors[h.ordinal()][Side.LEFT.ordinal()].command() == DoorCommand.NUDGE
					&& doorMotors[h.ordinal()][Side.RIGHT.ordinal()].command() == DoorCommand.NUDGE;
		}

		private void updateState(Hallway h)
		{
			//Initialize new state to the previous state.
			DoorState newState = state[h.ordinal()];

			switch (newState)
			{
				case CLOSED:
					numReversals = 0;
					if (allDoorMotorsClosing(h))
					{
						newState = DoorState.CLOSING;
					}
					break;
				case CLOSING:
					if (allDoorMotorsOpening(h))
					{
						newState = DoorState.REVERSED;
					}
					else if (allDoorMotorsNudge(h))
					{
						newState = DoorState.NUDGE;
					}
					break;
				case REVERSED:
					numReversals++;
					if (allDoorMotorsClosing(h))
					{
						newState = DoorState.CLOSING;
					}
					break;
				case NUDGE:
					if (allDoorMotorsStopped(h))
					{
						newState = DoorState.CLOSED;
					}
					break;
			}

			//Set the new state.
			state[h.ordinal()] = newState;
		}
	}


	@Override
	protected String[] summarize()
	{
		String[] arr = new String[7];
		arr[0] = "R-T.6 Violations: " + r6Violations;
		arr[1] = "R-T.7 Violations: " + r7Violations;
		arr[2] = "R-T.8.1 Violations: " + r8_1Violations;
		arr[3] = "R-T.8.2 Violations: " + r8_2Violations;
		arr[4] = "R-T.8.3 Violations: " + r8_3Violations;
		arr[5] = "R-T.9 Violations: " + r9Violations;
		arr[6] = "R-T.10 Violations: " + r10Violations;
		return arr;
	}
}
