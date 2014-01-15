/**
 * 18649 Fall 2013
 * Team 18
 * Yi Huo (yhuo)
 * Mustafa Bilgen (mbilgen)
 * Sairam Krishnan (sbkrishn)
 * Abhijit Lele (alele)
 * 
 * CHANGES:
 * Fixed mDriveSpeed direction
 * Fixed CurrentFloor logic (if not at a floor, it returns -1)
 * Removed speed checks for transitioning TO STATE_STOP
 */

package simulator.elevatorcontrol;

import jSimPack.SimTime;
import simulator.elevatormodules.CarLevelPositionCanPayloadTranslator;
import simulator.elevatormodules.DriveObject;
import simulator.elevatormodules.LevelingCanPayloadTranslator;
import simulator.elevatormodules.SafetySensorCanPayloadTranslator;
import simulator.framework.Controller;
import simulator.framework.Direction;
import simulator.framework.Hallway;
import simulator.framework.ReplicationComputer;
import simulator.framework.Speed;
import simulator.payloads.CanMailbox;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;
import simulator.payloads.DrivePayload;
import simulator.payloads.DrivePayload.WriteableDrivePayload;
import simulator.payloads.DriveSpeedPayload;
import simulator.payloads.DriveSpeedPayload.ReadableDriveSpeedPayload;
import simulator.elevatorcontrol.Utility.AtFloorArray;
import simulator.elevatorcontrol.Utility.DoorClosedArray;

/**
 * DriveControl Implementation
 * 
 * Input interface:
 * - mAtFloor[f,b]
 * - mDesiredFloor
 * - mDoorClosed[b,r]
 * - mlevel[d]
 * - mEmergencyBrake
 * 
 * Output interface:
 * - Drive[s,d]
 * - mDriveSpeed[f,b]
 * 
 * @author alele
 */

public class DriveControl extends Controller
{
	/***************************************************************************
	 * Declarations
	 **************************************************************************/
	//note that inputs are Readable objects, while outputs are Writeable objects

	//local physical state
	//Input Message
	private ReadableDriveSpeedPayload DriveSpeed;
	//Output Message
	private WriteableDrivePayload localdrive;

	//Network Interface
	//Input Messages

	//atFloor message
	private AtFloorArray mAtFloor;

	//door closed message
	private DoorClosedArray mDoorClosedFront;
	private DoorClosedArray mDoorClosedBack;


	//received desired floor message
	private ReadableCanMailbox networkDesiredFloorIn;
	// translator for the desired floor message
	private DesiredFloorCanPayloadTranslator mDesiredFloorIn;

	//receive emergency information
	private ReadableCanMailbox networkEmergencyBrakeIn;
	// translator for emergency message
	private SafetySensorCanPayloadTranslator mEmergencyBrakeIn;

	// receive Level informations
	private ReadableCanMailbox networkLevelUpIn, networkLevelDownIn;
	// translator for level message
	private LevelingCanPayloadTranslator mLevelUpIn, mLevelDownIn;

	// received car level position information
	private ReadableCanMailbox networkCarLevelPositionIn;
	// translator for car level Position
	private CarLevelPositionCanPayloadTranslator mCarLevelPositionIn;

	//Output Messages

	// set DriveSpeed information 
	private WriteableCanMailbox networkDriveSpeedOut;
	// translator for the DriveSpeed
	private DriveSpeedCanPayloadTranslator mDriveSpeedOut;

	// internal state variables
	private int Current_Floor;
	private int Desired_Floor;

	//additional internal state variables
	//private SimTime CountDown = SimTime.ZERO;

	//Variable to store period for controller arguments
	private SimTime period;


	//enumeration for the states of Drive controller for Sabath Mode design
	private enum State
	{
		DRIVE_STOP, DRIVE_MOVING, DRIVE_MOVING_FAST, DRIVE_Level_UP, DRIVE_Level_Down, EMERGENCY,
	}


	//State variable initialized to the initial state DOOR_CLOSED.
	private State state = State.DRIVE_STOP;


	/**
	 * The arguments listed in the .cf configuration file should match the order
	 * and
	 * type given here.
	 * 
	 * For your elevator controllers, you should make sure that the constructor
	 * matches
	 * the method signatures in ControllerBuilder.makeAll().
	 */
	public DriveControl(SimTime period, boolean verbose)
	{

		super("DriveControl", verbose);
		//super("DriveControl", true);
		//Storing controller arguments in the internal state variable
		this.period = period;

		/**
		 * Drive control initiated with a period
		 */
		log("Drive controller initiated with a period of :  ", period);

		/**
		 * Initialization of the state
		 */

		//physical object
		DriveSpeed = DriveSpeedPayload.getReadablePayload();
		physicalInterface.registerTimeTriggered(DriveSpeed);

		localdrive = DrivePayload.getWriteablePayload();
		physicalInterface.sendTimeTriggered(localdrive, period);

		//Network object
		// AtFloor-Array
		mAtFloor = new AtFloorArray(canInterface);

		// DoorClosed-Array
		mDoorClosedFront = new DoorClosedArray(Hallway.FRONT, canInterface);
		mDoorClosedBack = new DoorClosedArray(Hallway.BACK, canInterface);

		// EmergencyBrake CanMailbox
		networkEmergencyBrakeIn = CanMailbox
				.getReadableCanMailbox(MessageDictionary.EMERGENCY_BRAKE_CAN_ID);
		mEmergencyBrakeIn = new SafetySensorCanPayloadTranslator(networkEmergencyBrakeIn);
		canInterface.registerTimeTriggered(networkEmergencyBrakeIn);

		// DesiredFloor Can Mailbox
		networkDesiredFloorIn = CanMailbox
				.getReadableCanMailbox(MessageDictionary.DESIRED_FLOOR_CAN_ID);
		mDesiredFloorIn = new DesiredFloorCanPayloadTranslator(networkDesiredFloorIn);
		canInterface.registerTimeTriggered(networkDesiredFloorIn);

		//Level Message Can Mailbox
		networkLevelUpIn = CanMailbox.getReadableCanMailbox(MessageDictionary.LEVELING_BASE_CAN_ID
				+ ReplicationComputer.computeReplicationId(Direction.UP));
		mLevelUpIn = new LevelingCanPayloadTranslator(networkLevelUpIn, Direction.UP);
		canInterface.registerTimeTriggered(networkLevelUpIn);

		networkLevelDownIn = CanMailbox
				.getReadableCanMailbox(MessageDictionary.LEVELING_BASE_CAN_ID
						+ ReplicationComputer.computeReplicationId(Direction.DOWN));
		mLevelDownIn = new LevelingCanPayloadTranslator(networkLevelDownIn, Direction.DOWN);
		canInterface.registerTimeTriggered(networkLevelDownIn);

		// Car Level Position Message Can Mailbox
		networkCarLevelPositionIn = CanMailbox
				.getReadableCanMailbox(MessageDictionary.CAR_LEVEL_POSITION_CAN_ID);
		mCarLevelPositionIn = new CarLevelPositionCanPayloadTranslator(networkCarLevelPositionIn);
		canInterface.registerTimeTriggered(networkCarLevelPositionIn);

		// DriveSpeed Message Can Mailbox
		networkDriveSpeedOut = CanMailbox
				.getWriteableCanMailbox(MessageDictionary.DRIVE_SPEED_CAN_ID);
		mDriveSpeedOut = new DriveSpeedCanPayloadTranslator(networkDriveSpeedOut);
		canInterface.sendTimeTriggered(networkDriveSpeedOut, period);

		Current_Floor = 1;

		//Starting the timer for DriveController
		timer.start(period);

	}

	/**
	 * Executes the state chart of this controller
	 * 
	 * @param callbackData - data from previous call of this controller
	 */
	@Override
	public void timerExpired(Object callbackData)
	{
		State newState = state;
		int curr_floor;

		Desired_Floor = mDesiredFloorIn.getFloor();
        
        if (Desired_Floor == 0)
            Desired_Floor = 1;

		curr_floor = mAtFloor.getCurrentFloor();
		if (curr_floor != MessageDictionary.NONE)
			Current_Floor = curr_floor;

		double speed = DriveSpeed.speed();
		int pos = mCarLevelPositionIn.getPosition();
		Direction dir = DriveSpeed.direction();



		switch (state)
		{
			case DRIVE_STOP:
				localdrive.set(Speed.STOP, Direction.STOP);
				mDriveSpeedOut.set(speed, DriveSpeed.direction());

				//#Transition T 6.1
				if ((Current_Floor != Desired_Floor) && (mDoorClosedFront.getBothClosed())
						&& (mDoorClosedBack.getBothClosed()))
				{
					newState = State.DRIVE_MOVING;
				}//#Transition T 6.6
				else if ((mLevelDownIn.getValue() == false) && (mLevelUpIn.getValue() == true)
						&& speed <= DriveObject.SlowSpeed && dir != Direction.UP)
				{
					newState = State.DRIVE_Level_Down;
				}//#Transition T 6.4
				else if ((mLevelUpIn.getValue() == false) && (mLevelDownIn.getValue() == true)
						&& speed <= DriveObject.SlowSpeed && dir != Direction.DOWN)
				{
					newState = State.DRIVE_Level_UP;
				}
				else
				{
					newState = state;
				}
				break;

			case DRIVE_MOVING:
			    mDriveSpeedOut.set(speed, DriveSpeed.direction());
				if (Current_Floor < Desired_Floor)
				{
					localdrive.set(Speed.SLOW, Direction.UP);
				}
				if (Current_Floor > Desired_Floor)
				{
					localdrive.set(Speed.SLOW, Direction.DOWN);
				}
				//#Transition T6.3
				if (mEmergencyBrakeIn.getValue())
				{
					newState = State.EMERGENCY;
				}
				//#Transtion T6.9
				else if (Utility.getCommitPoint(pos, speed, dir, Desired_Floor) == true
						&& speed >= DriveObject.SlowSpeed)
				{
					newState = State.DRIVE_MOVING_FAST;
				}
				//#Transition T6.11
				else if (Current_Floor == Desired_Floor && (mLevelDownIn.getValue() == false)
						&& (mLevelUpIn.getValue() == true) && speed <= DriveObject.SlowSpeed
						&& dir == Direction.DOWN)
				{
					newState = State.DRIVE_Level_Down;
				}
				//#Transition T6.2
				else if (Current_Floor == Desired_Floor && (mLevelDownIn.getValue() == true)
						&& (mLevelUpIn.getValue() == false) && speed <= DriveObject.SlowSpeed
						&& dir == Direction.UP)
				{
					newState = State.DRIVE_Level_UP;
				}
				else
				{
					newState = state;
				}

				break;

			case EMERGENCY:
				localdrive.set(Speed.STOP, Direction.STOP);
				mDriveSpeedOut.set(speed, DriveSpeed.direction());
				newState = state;
				break;

			case DRIVE_Level_UP:
				localdrive.set(Speed.LEVEL, Direction.UP);
				mDriveSpeedOut.set(speed, DriveSpeed.direction());
				//#Transition T.6.5
				if (mLevelUpIn.getValue() == true)
				{
					newState = State.DRIVE_STOP;
				}
				else
				{
					newState = state;
				}
				break;

			case DRIVE_Level_Down:
				localdrive.set(Speed.LEVEL, Direction.DOWN);
				mDriveSpeedOut.set(speed, DriveSpeed.direction());
				//#Transition T.6.7
				if (mLevelDownIn.getValue() == true)
				{
					newState = State.DRIVE_STOP;
				}
				else
				{
					newState = state;
				}
				break;
			case DRIVE_MOVING_FAST:
				log(pos);
				if (Current_Floor < Desired_Floor)
				{
					localdrive.set(Speed.FAST, Direction.UP);
					mDriveSpeedOut.set(speed, DriveSpeed.direction());
				}
				if (Current_Floor > Desired_Floor)
				{
					localdrive.set(Speed.FAST, Direction.DOWN);
					mDriveSpeedOut.set(speed, DriveSpeed.direction());
				}
				//#Transition T6.10
				if (mEmergencyBrakeIn.getValue())
				{
					newState = State.EMERGENCY;
				}
				//#Transition T6.8
				else if (Utility.getCommitPoint(pos, speed, dir, Desired_Floor) == false)
				{
					newState = State.DRIVE_MOVING;
				}
				else
				{
					newState = state;
				}
				break;


			default:
				throw new RuntimeException("State " + state + " was not recognized.");
		}


		if (state == newState)
		{
			log("remains in state: ", state);
		}
		else
		{
			log("Transition:", state, "->", newState);
		}
		//update the state variable
		state = newState;
        
		//report the current state
		setState(STATE_KEY, newState.toString());

		//schedule the next iteration of the controller
		timer.start(period);

	}
}
