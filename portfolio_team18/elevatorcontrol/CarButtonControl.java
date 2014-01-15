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
import simulator.elevatormodules.DoorClosedCanPayloadTranslator;
import simulator.framework.Controller;
import simulator.framework.Direction;
import simulator.framework.Hallway;
import simulator.framework.ReplicationComputer;
import simulator.framework.Side;
import simulator.payloads.CanMailbox;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;
import simulator.payloads.CarCallPayload;
import simulator.payloads.CarCallPayload.ReadableCarCallPayload;
import simulator.payloads.CarLightPayload;
import simulator.payloads.CarLightPayload.WriteableCarLightPayload;
import simulator.payloads.translators.BooleanCanPayloadTranslator;
import simulator.payloads.translators.IntegerCanPayloadTranslator;
import simulator.payloads.CANNetwork;
import simulator.payloads.CANNetwork.CanConnection;
import simulator.elevatorcontrol.Utility;
import simulator.elevatorcontrol.Utility.DoorClosedArray;
import simulator.elevatorcontrol.Utility.AtFloorArray;

import java.lang.String;

/**
 * CarButtonControl Implementation
 * 
 * Input interface:
 * - mAtFloor[f,b] 
 * - mDesiredFloor
 * - mDoorClosed[b,r]
 * - CarCall[f,b]
 * 
 * Output interface:
 * - CarLight[f,b]
 * - mCarCall[f,b]
 * 
 * @author yhuo
 */
public class CarButtonControl extends Controller {

	// local physical state
	private ReadableCarCallPayload localCarCall;
	private WriteableCarLightPayload localCarLight;
	
	// network interface
	
	// send car call message
	private WriteableCanMailbox networkCarCallOut;
	// translator for the car call message
	private BooleanCanPayloadTranslator mCarCall;
	
	// received desired floor message
	private ReadableCanMailbox networkDesiredFloor;
	// translator for the desired floor message
	private IntegerCanPayloadTranslator mDesiredFloor;
	
	// at floor message
	private AtFloorArray mAtFloor;
	
	// door closed message
	private DoorClosedArray mDoorClosed;
	
	// variables to keep track of instance
	private final Hallway hallway;
	private final int floor;
	
	// internal state variables
	private int currentFloor;
	
	// store the period for the controller
	private SimTime period;
	
	// car light states
	private enum State {
		CAR_LIGHT_ON,
		CAR_LIGHT_OFF,
	}

	// state variable initialized to CAR_LIGHT_OFF
	private State state = State.CAR_LIGHT_OFF;	
	
	/**
	 * @param floor - the floor of the car button that this controller controls
	 * @param hallway - the hallway of the car button that this controller controls
	 * @param period - how often this controller runs
	 * @param verbose - whether this controller should print outputs
	 */
	public CarButtonControl(int floor, Hallway hallway, SimTime period, boolean verbose) {
		// call to superclass
		super("CarButtonControl" + ReplicationComputer.makeReplicationString(floor, hallway), verbose);
		
		// stored constructor arguments in internal state
		this.period = period;
		this.floor = floor;
		this.hallway = hallway;
		
		// Initialize physical state
		
		localCarCall = CarCallPayload.getReadablePayload(floor, hallway);
		physicalInterface.registerTimeTriggered(localCarCall);
		
		localCarLight = CarLightPayload.getWriteablePayload(floor, hallway);
		physicalInterface.sendTimeTriggered(localCarLight, period);
		
		// Initialize network interface
		
		networkCarCallOut = CanMailbox.getWriteableCanMailbox(MessageDictionary.CAR_CALL_BASE_CAN_ID + ReplicationComputer.computeReplicationId(floor, hallway));
		mCarCall = new BooleanCanPayloadTranslator(networkCarCallOut);
		canInterface.sendTimeTriggered(networkCarCallOut, period);
		
		networkDesiredFloor = CanMailbox.getReadableCanMailbox(MessageDictionary.DESIRED_FLOOR_CAN_ID + ReplicationComputer.computeReplicationId(floor, hallway));
		mDesiredFloor = new IntegerCanPayloadTranslator(networkDesiredFloor);
		canInterface.registerTimeTriggered(networkDesiredFloor);
		
		mAtFloor = new AtFloorArray(canInterface);
		
		mDoorClosed = new DoorClosedArray(hallway, canInterface);
		
		// Start controller
		timer.start(period);	
	}
	
	/**
	 * Executes the state chart of this controller
	 * 
	 * @param callbackData - data from previous call of this controller
	 */
	public void timerExpired(Object callbackData) {
		State newState = state;
		
		switch (state) {
			case CAR_LIGHT_OFF :
				// state actions for 'CAR_LIGHT_OFF'
				localCarLight.set(false);
				mCarCall.set(false);
				currentFloor = mAtFloor.getCurrentFloor();
				
				//#transition 'T.9.1'
				if(localCarCall.isPressed())
					newState = State.CAR_LIGHT_ON;
				
				break;
			case CAR_LIGHT_ON :
				// state actions for 'CAR_LIGHT_ON'
				localCarLight.set(true);
				mCarCall.set(true);
				currentFloor = mAtFloor.getCurrentFloor();
				
				//#transition 'T.9.2'
				if((currentFloor == floor) && !mDoorClosed.getBothClosed())
					newState = State.CAR_LIGHT_OFF;
				
				break;
			default : 
				throw new RuntimeException("State " + state + " was not recognized.");
		}
		
		// log the results of this iteration
		if(state == newState) {
		    log("remains in state: ",state);
		    log("mCarCall: ", mCarCall.getValue());
	    } else {
	        log("Transition:",state,"->",newState);
		    log("mCarCall: ", mCarCall.getValue());
	    }
		
		// update the state variable
		state = newState;
		
		//report the current state
        setState(STATE_KEY,newState.toString());
		
		// schedule the next iteration of the controller
		timer.start(period);
	}
}
