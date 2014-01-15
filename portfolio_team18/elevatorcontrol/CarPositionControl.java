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
import simulator.elevatorcontrol.Utility.*;
import simulator.elevatormodules.*;
import simulator.framework.*;
import simulator.payloads.*;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;
import simulator.payloads.CarPositionIndicatorPayload.WriteableCarPositionIndicatorPayload;

/**
 * CarPositionControl Implementation
 * 
 * Input interface:
 * - mAtFloor[f,b]
 * - mDesiredFloor
 * - mCarLevelPosition
 * - mDriveSpeed
 * 
 * Output interface:
 * - CarPositionIndicator
 * 
 * @author alele
 */
public class CarPositionControl extends Controller
{
    /* Note that all inputs are of type ReadableCanMailbox while outputs are of type WriteableCanMailbox. */

	//mAtFloor message
	private AtFloorArray mAtFloor;

	//received DesiredFloor network message
	private ReadableCanMailbox networkDesiredFloor;
	// translator for mDesiredFloor
	private DesiredFloorCanPayloadTranslator mDesiredFloor;

	//received CarLevelPosition message
	private ReadableCanMailbox networkCarLevelPosition;
	//translator for mCarLevelPosition
	private CarLevelPositionCanPayloadTranslator mCarLevelPosition;

       //received DriveSpeed message
       private ReadableCanMailbox networkDriveSpeed;
       //translator for mDriveSpeed
       private DriveSpeedCanPayloadTranslator mDriveSpeed;

	//Physical CarPositionIndicator Message (output)
	private WriteableCarPositionIndicatorPayload carPositionIndicator;

	// internal state variables
	private int currentPosition; //last floor passed by elevator - b/w 1 and 8 inclusive
	private int floor;
	private SimTime period; //period of controller

	//enumeration for the states of Drive controller for Sabath Mode design
	private enum State {
		AT_FLOOR,
		MOVING,
	}

	//State variable initialized to the initial state AT_FLOOR.
	private State state = State.AT_FLOOR;


	/**
	 * The arguments listed in the .cf configuration file should match the order
	 * and type given here.
	 * 
	 * For your elevator controllers, you should make sure that the constructor
	 * matches the method signatures in ControllerBuilder.makeAll().
	 */
	public CarPositionControl(SimTime period, boolean verbose)
	{
		// this indicates a call to the superclass controller
		super("CarPositionControl", verbose);

		// stored the constructor arguments in internal state
		this.period = period;
		//CarPosition control initiated with a period
		log("Car Position controller initiated with a period of :  ", period);

		//Initialize mAtFloor.
		mAtFloor = new AtFloorArray(canInterface);

		//Initialize mDesiredFloor.
		networkDesiredFloor = CanMailbox.getReadableCanMailbox(MessageDictionary.DESIRED_FLOOR_CAN_ID);
		mDesiredFloor = new DesiredFloorCanPayloadTranslator(networkDesiredFloor);
		canInterface.registerTimeTriggered(networkDesiredFloor);

		//Initialize mCarLevelPosition.
		networkCarLevelPosition = CanMailbox.getReadableCanMailbox(MessageDictionary.CAR_LEVEL_POSITION_CAN_ID);
		mCarLevelPosition = new CarLevelPositionCanPayloadTranslator(networkCarLevelPosition);
		canInterface.registerTimeTriggered(networkCarLevelPosition);

		//Initialize mDriveSpeed.
		networkDriveSpeed = CanMailbox.getReadableCanMailbox(MessageDictionary.DRIVE_SPEED_CAN_ID);
		mDriveSpeed = new DriveSpeedCanPayloadTranslator(networkDriveSpeed);
		canInterface.registerTimeTriggered(networkDriveSpeed);

		//Output physical message CarPositionIndicator.
		carPositionIndicator = CarPositionIndicatorPayload.getWriteablePayload();
		physicalInterface.sendTimeTriggered(carPositionIndicator, period);

		timer.start(period);
	}


	@Override
	public void timerExpired(Object callbackData)
	{
		State newState = state;
		int currentFloor = mAtFloor.getCurrentFloor();
		int desiredFloor = mDesiredFloor.getFloor();
		int carPosition = mCarLevelPosition.getPosition();
		double driveSpeed = mDriveSpeed.getSpeed();
		Direction dir = mDriveSpeed.getDirection();

		switch (state) {
			//#state 'S.10.1'
			case AT_FLOOR:
				//state 'S.10.1' actions
				carPositionIndicator.set(currentFloor);
				if (currentFloor != MessageDictionary.NONE) {
					currentPosition = currentFloor;
				}

				//#transition 'T.10.1'
				if (mDriveSpeed.getSpeed() > 0) {
					newState = State.MOVING;
				}
				else {
					newState = state;
				}
				break;
			//#state 'S.10.2'
			case MOVING:
				//state 'S.10.2' actions
				carPositionIndicator.set(currentPosition);

				if (currentFloor == MessageDictionary.NONE && desiredFloor > currentPosition && 
					dir == Direction.UP && currentPosition < 8 && 
					Utility.atCommitPoint(carPosition,dir,currentPosition+1)) {
						currentPosition = currentPosition + 1;
				}
				else if (currentFloor == MessageDictionary.NONE && desiredFloor < currentPosition && 
					dir == Direction.DOWN && currentPosition > 1 && 
					Utility.atCommitPoint(carPosition,dir,currentPosition-1)) {
						currentPosition = currentPosition - 1;
				}
				//#transition 'T.10.2'
				else if (currentFloor != MessageDictionary.NONE && mDriveSpeed.getSpeed() == 0) {
					newState = State.AT_FLOOR;
				}
				else {
					newState = state;
				}
				break;
			default:
				throw new RuntimeException("State " + state + " was not recognized.");

		}

		if (state == newState) {
			log("remains in state: ", state);
		}
		else {
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
