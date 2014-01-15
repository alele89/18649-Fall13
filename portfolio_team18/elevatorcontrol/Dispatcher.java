/**
 * 18649 Fall 2013
 * Team 18
 * Yi Huo (yhuo)
 * Mustafa Bilgen (mbilgen)
 * Sairam Krishnan (sbkrishn)
 * Abhijit Lele (alele)
 */

package simulator.elevatorcontrol;

import jSimPack.SimTime;
import simulator.elevatorcontrol.Utility;
import simulator.elevatorcontrol.Utility.AtFloorArray;
import simulator.elevatorcontrol.Utility.CarCallArray;
import simulator.elevatorcontrol.Utility.DoorClosedArray;
import simulator.elevatorcontrol.Utility.HallCallArray;
import simulator.elevatorcontrol.DesiredFloorCanPayloadTranslator;
import simulator.elevatorcontrol.DesiredDwellCanPayloadTranslator;
import simulator.elevatorcontrol.DriveSpeedCanPayloadTranslator;
import simulator.elevatormodules.CarLevelPositionCanPayloadTranslator;
import simulator.elevatormodules.DoorClosedCanPayloadTranslator;
import simulator.framework.Controller;
import simulator.framework.Direction;
import simulator.framework.Elevator;
import simulator.framework.Hallway;
import simulator.framework.ReplicationComputer;
import simulator.payloads.CanMailbox;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;

/**
 * Dispatcher Implementation
 * 
 * Input interface:
 * - mAtFloor[f, b]
 * - mDoorClosed[b, r]
 * - mHallCall[f, b, d]
 * - mCarCall[f, b]
 * - mCarWeight
 * - mDriveSpeed
 * - mCarLevelPosition
 * 
 * Output interface:
 * - mDesiredFloor
 * - mDesiredDwell[b]
 * 
 * @author mbilgen
 */

public class Dispatcher extends Controller
{

	// DECLARATIONS

	//Network interface
	private AtFloorArray atFloor;

	private DoorClosedArray frontDoorClosed;
	private DoorClosedArray backDoorClosed;

	private HallCallArray mHallCall;
	private CarCallArray mCarCall;

	/*
	 * UNUSED FOR NOW
	 * private ReadableCanMailbox networkCarWeight;
	 * private CarWeightCanPayloadTranslator mCarWeight;
	 * UNUSED FOR NOW
	 */

	private ReadableCanMailbox networkCarLevelPosition;
	private CarLevelPositionCanPayloadTranslator mCarLevelPosition;

	private ReadableCanMailbox networkDriveSpeed;
	private DriveSpeedCanPayloadTranslator mDriveSpeed;

	private WriteableCanMailbox networkDesiredFloor;
	private DesiredFloorCanPayloadTranslator mDesiredFloor;

	private WriteableCanMailbox[] networkDesiredDwell;
	private DesiredDwellCanPayloadTranslator[] mDesiredDwell;

	//Time
	private SimTime period;


	// Format for state names:
	// <what-elevator-is-doing>_<current-direction>_<next_direction>
	// for stopped state, CD=ND, so it's only repeated once.
	//State enumerations
	private enum State
	{
		STOPPED_UP, STOPPED_STOP, STOPPED_DOWN, MOVING_UP_UP, MOVING_UP_STOP, MOVING_UP_DOWN, MOVING_DOWN_UP, MOVING_DOWN_STOP, MOVING_DOWN_DOWN, EMERGENCY,
	}


	private State state = State.STOPPED_STOP;

	//State variables

	private int target;
	private int currentFloor;
	private Direction currentDirection;
	private Direction nextDirection;
	private SimTime countdown;

	//Useful constants
	public final int MIN_FLOOR = 1;
	public final int MAX_FLOOR;

	public final int DEFAULT_DWELL = 6; // seconds


	/**
	 * @param maxFloor The top floor
	 * @param period How often this controller should run
	 * @param verbose Whether this controller should spam outputs
	 */
	public Dispatcher(int maxFloor, SimTime period, boolean verbose)
	{
		super("Dispatcher", verbose);
		//super("Dispatcher", true);

		log("Created Dispatcher", " with period = ", period);

		MAX_FLOOR = maxFloor;
		this.period = period;

		networkDesiredDwell = new WriteableCanMailbox[2];
		mDesiredDwell = new DesiredDwellCanPayloadTranslator[2];

		// Initialize network interface

		backDoorClosed = new DoorClosedArray(Hallway.BACK, canInterface);
		frontDoorClosed = new DoorClosedArray(Hallway.FRONT, canInterface);

		mHallCall = new HallCallArray(maxFloor, canInterface);
		mCarCall = new CarCallArray(maxFloor, canInterface);

		atFloor = new AtFloorArray(canInterface);

		networkCarLevelPosition = CanMailbox
				.getReadableCanMailbox(MessageDictionary.CAR_LEVEL_POSITION_CAN_ID);
		mCarLevelPosition = new CarLevelPositionCanPayloadTranslator(networkCarLevelPosition);
		canInterface.registerTimeTriggered(networkCarLevelPosition);

		networkDriveSpeed = CanMailbox.getReadableCanMailbox(MessageDictionary.DRIVE_SPEED_CAN_ID);
		mDriveSpeed = new DriveSpeedCanPayloadTranslator(networkDriveSpeed);
		canInterface.registerTimeTriggered(networkDriveSpeed);

		networkDesiredFloor = CanMailbox
				.getWriteableCanMailbox(MessageDictionary.DESIRED_FLOOR_CAN_ID);
		mDesiredFloor = new DesiredFloorCanPayloadTranslator(networkDesiredFloor);
		canInterface.sendTimeTriggered(networkDesiredFloor, period);

		networkDesiredDwell[Hallway.FRONT.ordinal()] = CanMailbox
				.getWriteableCanMailbox(MessageDictionary.DESIRED_DWELL_BASE_CAN_ID
						+ ReplicationComputer.computeReplicationId(Hallway.FRONT));
		mDesiredDwell[Hallway.FRONT.ordinal()] = new DesiredDwellCanPayloadTranslator(
				networkDesiredDwell[Hallway.FRONT.ordinal()], Hallway.FRONT);
		canInterface.sendTimeTriggered(networkDesiredDwell[Hallway.FRONT.ordinal()], period);

		networkDesiredDwell[Hallway.BACK.ordinal()] = CanMailbox
				.getWriteableCanMailbox(MessageDictionary.DESIRED_DWELL_BASE_CAN_ID
						+ ReplicationComputer.computeReplicationId(Hallway.BACK));
		mDesiredDwell[Hallway.BACK.ordinal()] = new DesiredDwellCanPayloadTranslator(
				networkDesiredDwell[Hallway.BACK.ordinal()], Hallway.BACK);
		canInterface.sendTimeTriggered(networkDesiredDwell[Hallway.BACK.ordinal()], period);

		target = 1;

		state = State.STOPPED_STOP;

		countdown = SimTime.ZERO;

		timer.start(period);
	}

	private static int min(int a, int b)
	{
		if (a < b)
			return a;
		return b;
	}

	private static int max(int a, int b)
	{
		if (a > b)
			return a;
		return b;
	}

	public void timerExpired(Object callbackData)
	{
		State newState;
		Hallway desiredHallway = Hallway.NONE;
		int newTarget = target;
		boolean transUp;
		boolean transDown;

		int curFloor = atFloor.getCurrentFloor();
		if (curFloor != MessageDictionary.NONE)
			currentFloor = curFloor;

		int position = mCarLevelPosition.getPosition();
		double speed = mDriveSpeed.getSpeed();

		// Note that output messages are set *after* the switch statement.
		// Only target, nextDirection and desiredHallway are set here.

		switch (state)
		{
			case MOVING_UP_UP:
				//State actions for MOVING_UP_UP
				currentDirection = Direction.UP;
				nextDirection = Direction.UP;
				countdown = SimTime.ZERO;

				// Choose new target
				newTarget = target;
				for (int f = 1; f <= MAX_FLOOR; f++)
				{
					if (Utility.getCommitPoint(position, speed, currentDirection, f)
							&& (mHallCall.getAnyHallway(f, currentDirection) || mCarCall.getAny(f)))

						newTarget = min(newTarget, f);
				}

				// Choose desiredHallway
				if (mHallCall.get(newTarget, Hallway.FRONT, currentDirection)
						|| mCarCall.get(newTarget, Hallway.FRONT))
					desiredHallway = Hallway.FRONT;

				if (mHallCall.get(newTarget, Hallway.BACK, currentDirection)
						|| mCarCall.get(newTarget, Hallway.BACK))
				{

					if (desiredHallway == Hallway.FRONT)
						desiredHallway = Hallway.BOTH;
					else
						desiredHallway = Hallway.BACK;
				}

				//if (desiredHallway == Hallway.NONE)
				//	throw new RuntimeException("Dispatcher picked target with no calls: "
				//			+ newTarget);

				if ((atFloor.isAtFloor(newTarget, Hallway.FRONT) || atFloor.isAtFloor(newTarget,
						Hallway.BACK))
						&& !(frontDoorClosed.getBothClosed() && backDoorClosed.getBothClosed()))
					//#transition 'T.11.1'
					newState = State.STOPPED_UP;
				else
					newState = state;

				break;

			case MOVING_UP_STOP:
				//State actions for MOVING_UP_STOP
				currentDirection = Direction.UP;
				nextDirection = Direction.STOP;
				countdown = SimTime.ZERO;

				// Choose new target
				newTarget = Integer.MAX_VALUE;
				for (int f = 1; f <= MAX_FLOOR; f++)
				{
					if (Utility.getCommitPoint(position, speed, currentDirection, f)
							&& (mHallCall.getAnyHallway(f, currentDirection) || mCarCall.getAny(f)))

						newTarget = min(newTarget, f);
				}

				if (newTarget == Integer.MAX_VALUE)
				{
					newTarget = -1;
					for (int f = 1; f <= MAX_FLOOR; f++)
					{
						if (Utility.getCommitPoint(position, speed, currentDirection, f)
								&& (mHallCall.getAnyHallway(f, Direction.DOWN) || mCarCall
										.getAny(f)))

							newTarget = max(newTarget, f);
					}
				}

				if (newTarget == -1)
					newTarget = target;

				// Choose desiredHallway
				if (mCarCall.get(newTarget, Hallway.FRONT))
					desiredHallway = Hallway.FRONT;

				if (mCarCall.get(newTarget, Hallway.BACK))
				{
					if (desiredHallway == Hallway.FRONT)
						desiredHallway = Hallway.BOTH;
					else
						desiredHallway = Hallway.BACK;
				}

				// No RuntimeException stuff here, because the design allows 
				// mDesiredFloor.b to be inconsistent while changing state.

				// Do some complicated guard checks first:
				transUp = mHallCall.getAnyHallway(newTarget, Direction.UP);
				transDown = mHallCall.getAnyHallway(newTarget, Direction.DOWN);

				for (int f = newTarget + 1; f <= MAX_FLOOR; f++)
				{
					transUp |= mHallCall.getAny(f);
					transUp |= mCarCall.getAny(f);
				}

				for (int f = 1; f < newTarget; f++)
				{
					transDown |= mHallCall.getAny(f);
					transDown |= mCarCall.getAny(f);
				}

				if ((atFloor.isAtFloor(newTarget, Hallway.FRONT) || atFloor.isAtFloor(newTarget,
						Hallway.BACK))
						&& !(frontDoorClosed.getBothClosed() && backDoorClosed.getBothClosed()))
					//#transition 'T.11.2'
					newState = State.STOPPED_STOP;
				else if (transUp)
					//#transition 'T.11.11'
					newState = State.MOVING_UP_UP;
				else if (transDown)
					//#transition 'T.11.12'
					newState = State.MOVING_UP_DOWN;
				else
					newState = state;

				break;

			case MOVING_UP_DOWN:
				//State actions for MOVING_UP_DOWN
				currentDirection = Direction.UP;
				nextDirection = Direction.DOWN;
				countdown = SimTime.ZERO;

				// Choose new target
				newTarget = Integer.MAX_VALUE;
				for (int f = 1; f <= MAX_FLOOR; f++)
				{
					if (Utility.getCommitPoint(position, speed, currentDirection, f)
							&& (mHallCall.getAnyHallway(f, currentDirection) || mCarCall.getAny(f)))

						newTarget = min(newTarget, f);
				}

				if (newTarget == Integer.MAX_VALUE)
				{
					// No transitioning to MOVING_UP_UP state 
					// continue with nextDirection = DOWN

					// TODO: FIX STATECHART LOGIC!
					// special case for approaching floor and getting past
					// commit point
					if (mCarCall.getAny(target))
						newTarget = target;

					else
					{
						newTarget = -1;
						for (int f = 1; f <= MAX_FLOOR; f++)
						{
							if (mHallCall.getAnyHallway(f, Direction.DOWN) || mCarCall.getAny(f))

								newTarget = max(newTarget, f);
						}
					}
				}

				//if (newTarget == -1)
				//	throw new RuntimeException(
				//			"Dispatcher moving up with no pending calls: target = " + target);

				// Choose desiredHallway
				if (mCarCall.get(newTarget, Hallway.FRONT)
						|| mHallCall.get(newTarget, Hallway.FRONT, nextDirection))
					desiredHallway = Hallway.FRONT;

				if (mCarCall.get(newTarget, Hallway.BACK)
						|| mHallCall.get(newTarget, Hallway.BACK, nextDirection))
				{
					if (desiredHallway == Hallway.FRONT)
						desiredHallway = Hallway.BOTH;
					else
						desiredHallway = Hallway.BACK;
				}
				// No RuntimeException stuff here, because the design allows 
				// mDesiredFloor.b to be inconsistent while changing state.

				// Do some complicated guard checks first:
				transUp = mHallCall.getAnyHallway(newTarget, Direction.UP);

				for (int f = newTarget + 1; f <= MAX_FLOOR; f++)
				{
					transUp |= mHallCall.getAny(f);
					transUp |= mCarCall.getAny(f);
				}

				if ((atFloor.isAtFloor(newTarget, Hallway.FRONT) || atFloor.isAtFloor(newTarget,
						Hallway.BACK))
						&& !(frontDoorClosed.getBothClosed() && backDoorClosed.getBothClosed()))
					//#transition 'T.11.3'
					newState = State.STOPPED_DOWN;
				else if (transUp)
					//#transition 'T.11.13'
					newState = State.MOVING_UP_UP;
				else
					newState = state;
				break;

			case STOPPED_UP:
				//State actions for STOPPED_UP
				currentDirection = Direction.UP;
				nextDirection = Direction.UP;

				// Choose new target
				newTarget = Integer.MAX_VALUE;
				for (int f = currentFloor; f <= MAX_FLOOR; f++)
				{
					if (mHallCall.getAnyHallway(f, currentDirection) || mCarCall.getAny(f))

						newTarget = min(newTarget, f);
				}

				if (newTarget == Integer.MAX_VALUE)
				{
					newTarget = -1;
					for (int f = currentFloor + 1; f <= MAX_FLOOR; f++)
					{
						if (mHallCall.getAnyHallway(f, Direction.DOWN) || mCarCall.getAny(f))

							newTarget = max(newTarget, f);
					}

					// Choose desiredHallway
					if (mHallCall.get(newTarget, Hallway.FRONT, Direction.DOWN))
						desiredHallway = Hallway.FRONT;

					if (mHallCall.get(newTarget, Hallway.BACK, Direction.DOWN))
					{
						if (desiredHallway == Hallway.FRONT)
							desiredHallway = Hallway.BOTH;
						else
							desiredHallway = Hallway.BACK;
					}

					if (newTarget == -1)
						newTarget = target;

				}
				else
				{
					// Choose desiredHallway
					if (mCarCall.get(newTarget, Hallway.FRONT)
							|| mHallCall.get(newTarget, Hallway.FRONT, Direction.UP))
						desiredHallway = Hallway.FRONT;

					if (mCarCall.get(newTarget, Hallway.BACK)
							|| mHallCall.get(newTarget, Hallway.BACK, Direction.UP))
					{
						if (desiredHallway == Hallway.FRONT)
							desiredHallway = Hallway.BOTH;
						else
							desiredHallway = Hallway.BACK;
					}

				}

				if (frontDoorClosed.getBothClosed() && backDoorClosed.getBothClosed())
					countdown = SimTime.add(countdown, period);

				if (frontDoorClosed.getBothClosed() && backDoorClosed.getBothClosed()
						&& speed > 0.06)
					//#transition 'T.11.9'
					newState = State.MOVING_UP_STOP;
				else if (countdown.isGreaterThan(new SimTime(DEFAULT_DWELL,
						SimTime.SimTimeUnit.SECOND)))
					//#transition 'T.11.18'
					newState = State.STOPPED_STOP;
				else
					newState = state;

				break;

			case STOPPED_STOP:
				//State actions for STOPPED_STOP
				currentDirection = Direction.STOP;
				nextDirection = Direction.STOP;
				newTarget = currentFloor;
				desiredHallway = Hallway.NONE;
				countdown = SimTime.ZERO;

				transUp = mHallCall.getAnyHallway(currentFloor, Direction.UP);
				transDown = mHallCall.getAnyHallway(currentFloor, Direction.DOWN);
				// Do some complicated guard checks first:
				for (int f = currentFloor + 1; f <= MAX_FLOOR; f++)
				{
					if (mHallCall.getAny(f) || mCarCall.getAny(f))
						transUp = true;
				}
				for (int f = 1; f < currentFloor; f++)
				{
					if (mHallCall.getAny(f) || mCarCall.getAny(f))
						transDown = true;
				}

                if (!transUp && !transDown)
                {
                    // Staying at this floor,
                    // check for CarCalls
                    if (mCarCall.get(currentFloor, Hallway.FRONT))
                        desiredHallway = Hallway.FRONT;
                    if (mCarCall.get(currentFloor, Hallway.BACK))
                    {
                        if (desiredHallway == Hallway.FRONT)
                            desiredHallway = Hallway.BOTH;
                        else
                            desiredHallway = Hallway.BACK;
                    }
                }

				if (transUp)
					//#transition 'T.11.7'
					newState = State.STOPPED_UP;
				else if (transDown)
					//#transition 'T.11.8'
					newState = State.STOPPED_DOWN;
				else
					newState = state;
				break;

			case STOPPED_DOWN:
				//State actions for STOPPED_DOWN
				currentDirection = Direction.DOWN;
				nextDirection = Direction.DOWN;

				// Choose new target
				newTarget = -1;
				for (int f = 1; f <= currentFloor; f++)
				{
					if (mHallCall.getAnyHallway(f, currentDirection) || mCarCall.getAny(f))

						newTarget = max(newTarget, f);
				}

				if (newTarget == -1)
				{
					newTarget = Integer.MAX_VALUE;
					for (int f = 1; f < currentFloor; f++)
					{
						if (mHallCall.getAnyHallway(f, Direction.UP) || mCarCall.getAny(f))

							newTarget = min(newTarget, f);
					}

					// Choose desiredHallway
					if (mHallCall.get(newTarget, Hallway.FRONT, Direction.UP))
						desiredHallway = Hallway.FRONT;

					if (mHallCall.get(newTarget, Hallway.BACK, Direction.UP))
					{
						if (desiredHallway == Hallway.FRONT)
							desiredHallway = Hallway.BOTH;
						else
							desiredHallway = Hallway.BACK;
					}

					if (newTarget == Integer.MAX_VALUE)
						newTarget = target;

				}
				else
				{
					// Choose desiredHallway
					if (mCarCall.get(newTarget, Hallway.FRONT)
							|| mHallCall.get(newTarget, Hallway.FRONT, Direction.DOWN))
						desiredHallway = Hallway.FRONT;

					if (mCarCall.get(newTarget, Hallway.BACK)
							|| mHallCall.get(newTarget, Hallway.BACK, Direction.DOWN))
					{
						if (desiredHallway == Hallway.FRONT)
							desiredHallway = Hallway.BOTH;
						else
							desiredHallway = Hallway.BACK;
					}

				}
				if (frontDoorClosed.getBothClosed() && backDoorClosed.getBothClosed())
					countdown = SimTime.add(countdown, period);

				if (frontDoorClosed.getBothClosed() && backDoorClosed.getBothClosed()
						&& speed > 0.06)

					//#transition 'T.11.10'
					newState = State.MOVING_DOWN_STOP;
				else if (countdown.isGreaterThan(new SimTime(DEFAULT_DWELL,
						SimTime.SimTimeUnit.SECOND)))
					//#transition 'T.11.19'
					newState = State.STOPPED_STOP;
				else
					newState = state;

				break;

			case MOVING_DOWN_UP:
				//State actions for MOVING_DOWN_UP
				currentDirection = Direction.DOWN;
				nextDirection = Direction.UP;
				countdown = SimTime.ZERO;

				// Choose new target
				newTarget = -1;
				for (int f = 1; f <= MAX_FLOOR; f++)
				{
					if (Utility.getCommitPoint(position, speed, currentDirection, f)
							&& (mHallCall.getAnyHallway(f, currentDirection) || mCarCall.getAny(f)))

						newTarget = max(newTarget, f);
				}

				if (newTarget == -1)
				{
					// No transitioning to MOVING_UP_UP state 
					// continue with nextDirection = DOWN

					// TODO: FIX STATECHART LOGIC!
					// special case for approaching floor and getting past
					// commit point
					if (mCarCall.getAny(target))
						newTarget = target;

					else
					{
						newTarget = Integer.MAX_VALUE;
						for (int f = 1; f <= MAX_FLOOR; f++)
						{
							if (mHallCall.getAnyHallway(f, Direction.UP) || mCarCall.getAny(f))

								newTarget = min(newTarget, f);
						}
					}
				}

				//if (newTarget == Integer.MAX_VALUE)
				//	throw new RuntimeException(
				//			"Dispatcher moving up with no pending calls: target = " + target);

				// Choose desiredHallway
				if (mCarCall.get(newTarget, Hallway.FRONT)
						|| mHallCall.get(newTarget, Hallway.FRONT, nextDirection))
					desiredHallway = Hallway.FRONT;

				if (mCarCall.get(newTarget, Hallway.BACK)
						|| mHallCall.get(newTarget, Hallway.BACK, nextDirection))
				{
					if (desiredHallway == Hallway.FRONT)
						desiredHallway = Hallway.BOTH;
					else
						desiredHallway = Hallway.BACK;
				}
				// No RuntimeException stuff here, because the design allows 
				// mDesiredFloor.b to be inconsistent while changing state.

				// Do some complicated guard checks first:
				transDown = mHallCall.getAnyHallway(newTarget, Direction.DOWN);

				for (int f = 1; f < newTarget; f++)
				{
					transDown |= mHallCall.getAny(f);
					transDown |= mCarCall.getAny(f);
				}

				if ((atFloor.isAtFloor(newTarget, Hallway.FRONT) || atFloor.isAtFloor(newTarget,
						Hallway.BACK))
						&& !(frontDoorClosed.getBothClosed() && backDoorClosed.getBothClosed()))
					//#transition 'T.11.4'
					newState = State.STOPPED_UP;
				else if (transDown)
					//#transition 'T.11.16'
					newState = State.MOVING_DOWN_DOWN;
				else
					newState = state;
				break;

			case MOVING_DOWN_STOP:
				//State actions for MOVING_DOWN_STOP
				currentDirection = Direction.DOWN;
				nextDirection = Direction.STOP;
				countdown = SimTime.ZERO;

				// Choose new target
				newTarget = -1;
				for (int f = 1; f <= MAX_FLOOR; f++)
				{
					if (Utility.getCommitPoint(position, speed, currentDirection, f)
							&& (mHallCall.getAnyHallway(f, currentDirection) || mCarCall.getAny(f)))

						newTarget = max(newTarget, f);
				}

				if (newTarget == -1)
				{
					newTarget = Integer.MAX_VALUE;
					for (int f = 1; f <= MAX_FLOOR; f++)
					{
						if (Utility.getCommitPoint(position, speed, currentDirection, f)
								&& (mHallCall.getAnyHallway(f, Direction.UP) || mCarCall.getAny(f)))

							newTarget = min(newTarget, f);
					}
				}

				if (newTarget == Integer.MAX_VALUE)
					newTarget = target;

				// Choose desiredHallway
				if (mCarCall.get(newTarget, Hallway.FRONT))
					desiredHallway = Hallway.FRONT;

				if (mCarCall.get(newTarget, Hallway.BACK))
				{
					if (desiredHallway == Hallway.FRONT)
						desiredHallway = Hallway.BOTH;
					else
						desiredHallway = Hallway.BACK;
				}

				// No RuntimeException stuff here, because the design allows 
				// mDesiredFloor.b to be inconsistent while changing state.

				// Do some complicated guard checks first:
				transUp = mHallCall.getAnyHallway(newTarget, Direction.UP);
				transDown = mHallCall.getAnyHallway(newTarget, Direction.DOWN);

				for (int f = newTarget + 1; f <= MAX_FLOOR; f++)
				{
					transUp |= mHallCall.getAny(f);
					transUp |= mCarCall.getAny(f);
				}

				for (int f = 1; f < newTarget; f++)
				{
					transDown |= mHallCall.getAny(f);
					transDown |= mCarCall.getAny(f);
				}

				if ((atFloor.isAtFloor(newTarget, Hallway.FRONT) || atFloor.isAtFloor(newTarget,
						Hallway.BACK))
						&& !(frontDoorClosed.getBothClosed() && backDoorClosed.getBothClosed()))
					//#transition 'T.11.5'
					newState = State.STOPPED_STOP;
				else if (transDown)
					//#transition 'T.11.15'
					newState = State.MOVING_DOWN_DOWN;
				else if (transUp)
					//#transition 'T.11.14'
					newState = State.MOVING_DOWN_UP;
				else
					newState = state;

				break;

			case MOVING_DOWN_DOWN:
				//State actions for MOVING_DOWN_DOWN
				currentDirection = Direction.DOWN;
				nextDirection = Direction.DOWN;
				countdown = SimTime.ZERO;

				// Choose new target
				newTarget = target;
				for (int f = 1; f <= MAX_FLOOR; f++)
				{
					if (Utility.getCommitPoint(position, speed, currentDirection, f)
							&& (mHallCall.getAnyHallway(f, currentDirection) || mCarCall.getAny(f)))

						newTarget = max(newTarget, f);
				}

				// Choose desiredHallway
				if (mHallCall.get(newTarget, Hallway.FRONT, currentDirection)
						|| mCarCall.get(newTarget, Hallway.FRONT))
					desiredHallway = Hallway.FRONT;

				if (mHallCall.get(newTarget, Hallway.BACK, currentDirection)
						|| mCarCall.get(newTarget, Hallway.BACK))
				{

					if (desiredHallway == Hallway.FRONT)
						desiredHallway = Hallway.BOTH;
					else
						desiredHallway = Hallway.BACK;
				}

				//if (desiredHallway == Hallway.NONE)
				//	throw new RuntimeException("Dispatcher picked target with no calls: "
				//			+ newTarget);

				if ((atFloor.isAtFloor(newTarget, Hallway.FRONT) || atFloor.isAtFloor(newTarget,
						Hallway.BACK))
						&& !(frontDoorClosed.getBothClosed() && backDoorClosed.getBothClosed()))
					//#transition 'T.11.6'
					newState = State.STOPPED_DOWN;
				else
					newState = state;
				break;

			case EMERGENCY:
				//State actions for EMERGENCY
				target = 1;
				newTarget = 1;
				currentDirection = Direction.STOP;
				nextDirection = Direction.STOP;
				desiredHallway = Hallway.NONE;

				newState = state;
				break;

			default:
				throw new RuntimeException("Invalid state: " + state);
		}

		if ((!backDoorClosed.getBothClosed() || !frontDoorClosed.getBothClosed())
				&& (atFloor.getCurrentFloor() == MessageDictionary.NONE))
		{
			//#transition 'T.11.17'
			newState = State.EMERGENCY;
		}

		if (target != newTarget)
			log("target changed from ", target, " to ", newTarget);
			//System.out.println("target changed from " + target + " to " + newTarget);
		target = newTarget;

		// state actions common to all states
		mDesiredFloor.set(target, nextDirection, desiredHallway);
		mDesiredDwell[Hallway.FRONT.ordinal()].set(DEFAULT_DWELL);
		mDesiredDwell[Hallway.BACK.ordinal()].set(DEFAULT_DWELL);

		if (state == newState)
			log("remains in state: ", state);
			//System.out.println("remains in state: " + state);
		else
			log("Transition: ", state, "->", newState);
			//System.out.println("Transition: " + state + "->" + newState);

		state = newState;

		setState(STATE_KEY, newState.toString());

		timer.start(period);
	}

}
