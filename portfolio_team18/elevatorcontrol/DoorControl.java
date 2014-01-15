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
import simulator.elevatormodules.*;
import simulator.framework.*;
import simulator.payloads.*;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;
import simulator.payloads.DoorMotorPayload.WriteableDoorMotorPayload;
import simulator.payloads.translators.BooleanCanPayloadTranslator;
import simulator.elevatorcontrol.Utility;
import simulator.elevatorcontrol.Utility.*;

/**
 * DoorControl Implementation
 *
 * Inputs:
 *  mAtFloor[f,b]
 *  mDriveSpeed
 *  mDesiredFloor
 *  mDesiredDwell[b]
 *  mDoorClosed[b,r]
 *  mDoorOpened[b,r]
 *  mDoorReversal[b,r]
 *  mCarCall[f,b] - Not Used
 *  mHallCall[f,b,d] - Not Used
 *  mCarWeight
 *
 * Outputs:
 *  DoorMotor[b,r]
 *
 * @author Sairam Krishnan (sbkrishn)
 */
public class DoorControl extends Controller {

    /***************************************************************************
     * Declarations
     **************************************************************************/
    //note that inputs are Readable objects, while outputs are Writeable object

    //local physical state for the DoorMotor
    private WriteableDoorMotorPayload localDoorMotor;

    //DoorClosed message
    private ReadableCanMailbox networkDoorClosed;
    private DoorClosedCanPayloadTranslator mDoorClosed;
    
    //DoorOpened message
    private ReadableCanMailbox networkDoorOpened;
    private DoorOpenedCanPayloadTranslator mDoorOpened;

    //DoorReversal message
    private DoorReversalArray mDoorReversal;

    //AtFloor message
    private AtFloorArray mAtFloor; 

    //CarWeight message
    private ReadableCanMailbox networkCarWeight;
    private CarWeightCanPayloadTranslator mCarWeight;

    //DesiredFloor message
    private ReadableCanMailbox networkDesiredFloor;
    private DesiredFloorCanPayloadTranslator mDesiredFloor;

    //DriveSpeed message
    private ReadableCanMailbox networkDriveSpeed;
    private DriveSpeedCanPayloadTranslator mDriveSpeed;

    //DesiredDwell message
    private ReadableCanMailbox networkDesiredDwell;
    private DesiredDwellCanPayloadTranslator mDesiredDwell;

    //these variables keep track of which instance this is.
    private final Hallway hallway;
    private final Side side;
    
    //internal state variables
    private SimTime period;  //store the period for the controller
    private SimTime countDown = SimTime.ZERO; //Door open timeout - time to close doors if 0
    private boolean reversedOnce = false; //True if reversed once at current floor

    //enumerate states
    private enum State {
        DOOR_CLOSED,
        DOOR_OPENING,
        DOOR_OPEN,
        DOOR_CLOSING,
        DOOR_REVERSED,
        DOOR_NUDGE
    }

    //State variable initialized to the initial state DOOR_CLOSED.
    private State state = State.DOOR_CLOSED;

    /**
     * The arguments listed in the .cf configuration file should match the order and
     * type given here.
     *
     * For your elevator controllers, you should make sure that the constructor matches
     * the method signatures in ControllerBuilder.makeAll().
     */
    public DoorControl(Hallway hallway, Side side, SimTime period, boolean verbose) {
        //call to the Controller superclass constructor is required
        super("DoorControl" + ReplicationComputer.makeReplicationString(hallway, side), verbose);
        
        //stored the constructor arguments in internal state
        this.hallway = hallway;
        this.side = side;
        this.period = period;

        /* 
         * The log() method is inherited from the Controller class.  It takes an
         * array of objects which will be converted to strings and concatenated
         * only if the log message is actually written.  
         * 
         * For performance reasons, call with comma-separated lists, e.g.:
         *   log("object=",object);
         * Do NOT call with concatenated objects like:
         *   log("object=" + object);
         */
        log("Created DoorControl with period = ", period);
        
        //Create a Payload object for the left and right DoorMotors on this hallway.
        //This is an output, so it is created with the Writeable static factory method
        localDoorMotor = DoorMotorPayload.getWriteablePayload(hallway, side);
        //Register the payload to be sent periodically -- whatever value is stored
        //in the localDoorMotor objects will be sent out periodically with the period
        //specified by the period parameter.
        physicalInterface.sendTimeTriggered(localDoorMotor, period);

        /*
         * Registration for network messages.
         * (a) Create a CanMailbox - this object has the binary representation of the 
         * message data. 
         * (b) To register for network messages from the smart sensors or 
         * other objects defined in elevator modules, use the translators already 
         * defined in elevatormodules package. These translators are specific to
         * one type of message.
         * (c) Register to receive periodic updates to the mailbox via the CAN network.
         * The period of updates will be determined by the sender of the message.
         */
        int index;

        //Registration for DoorClosed message
        index = MessageDictionary.DOOR_CLOSED_SENSOR_BASE_CAN_ID;
        index += ReplicationComputer.computeReplicationId(hallway, side);
        networkDoorClosed = CanMailbox.getReadableCanMailbox(index);
        mDoorClosed = new DoorClosedCanPayloadTranslator(networkDoorClosed, hallway, side);
        canInterface.registerTimeTriggered(networkDoorClosed);

        //Registration for DoorOpened message 
        index = MessageDictionary.DOOR_OPEN_SENSOR_BASE_CAN_ID;
        index += ReplicationComputer.computeReplicationId(hallway, side);
        networkDoorOpened = CanMailbox.getReadableCanMailbox(index);
        mDoorOpened = new DoorOpenedCanPayloadTranslator(networkDoorOpened, hallway, side);
        canInterface.registerTimeTriggered(networkDoorOpened);

        //Registration for AtFloor message (using Utility.AtFloorArray)
        mAtFloor = new AtFloorArray(canInterface);

        //Registration for the DoorReversal messages
        mDoorReversal = new DoorReversalArray(hallway, canInterface);

        //Registration for the CarWeight message
        networkCarWeight = CanMailbox.getReadableCanMailbox(MessageDictionary.CAR_WEIGHT_CAN_ID);
        mCarWeight = new CarWeightCanPayloadTranslator(networkCarWeight);
        canInterface.registerTimeTriggered(networkCarWeight);

        //Registration for the DesiredFloor message
        networkDesiredFloor = CanMailbox.getReadableCanMailbox(MessageDictionary.DESIRED_FLOOR_CAN_ID);
        mDesiredFloor = new DesiredFloorCanPayloadTranslator(networkDesiredFloor);
        canInterface.registerTimeTriggered(networkDesiredFloor);

        //Registration for the DriveSpeed message
        networkDriveSpeed = CanMailbox.getReadableCanMailbox(MessageDictionary.DRIVE_SPEED_CAN_ID);
        mDriveSpeed = new DriveSpeedCanPayloadTranslator(networkDriveSpeed);
        canInterface.registerTimeTriggered(networkDriveSpeed);

        //Registration for the DesiredDwell message
        index = MessageDictionary.DESIRED_DWELL_BASE_CAN_ID;
        index += ReplicationComputer.computeReplicationId(hallway);
        networkDesiredDwell = CanMailbox.getReadableCanMailbox(index);
        mDesiredDwell = new DesiredDwellCanPayloadTranslator(networkDesiredDwell, hallway);
        canInterface.registerTimeTriggered(networkDesiredDwell);

        /* issuing the timer start method with no callback data means a NULL value 
         * will be passed to the callback later.  Use the callback data to distinguish
         * callbacks from multiple calls to timer.start() (e.g. if you have multiple
         * timers). 
         */
        timer.start(period);
    }

    /*
     * The timer callback is where the main controller code is executed.  For time
     * triggered design, this consists mainly of a switch block with a case blcok for
     * each state.  Each case block executes actions for that state, then executes
     * a transition to the next state if the transition conditions are met.
     */
    public void timerExpired(Object callbackData) {
        State newState = state;
        SimTime Dwell = new SimTime(mDesiredDwell.getValue(), SimTime.SimTimeUnit.SECOND); 

        switch (state) {
            //#state '5.1'
            case DOOR_CLOSED:
                //#state '5.1' actions
                localDoorMotor.set(DoorCommand.STOP); //DoorMotor[hallway,side] = STOP
                countDown = SimTime.ZERO; //CountDown initialized to 0 since this is reset state
                reversedOnce = false; //Reset state, so doors haven't been reversed yet

                //Check whether mDesiredFloor was set yet - it might not be set
                //when the elevator first starts up.
                int desiredFloor = mDesiredFloor.getFloor();
                if (desiredFloor == 0) {
	               break;
		        }
                
                //#transition 'T.5.1'
                Hallway desiredHallway = mDesiredFloor.getHallway();
                boolean atFloor = mAtFloor.isAtFloor(desiredFloor, hallway) && 
                    (desiredHallway == hallway || desiredHallway == Hallway.BOTH);

                if (atFloor && (mDriveSpeed.getSpeed() == 0.0 || mDriveSpeed.getDirection() == Direction.STOP)) {
                    newState = State.DOOR_OPENING;
                } 
                else {
                    newState = state;
                }
                break;
            //#state '5.2'
            case DOOR_OPENING:
                //#state '5.2' actions
                localDoorMotor.set(DoorCommand.OPEN); //DoorMotor[hallway, side] = OPEN
                countDown = Dwell; //Set CountDown to Dwell (the # of seconds to leave the door open).

                //#transition 'T.5.2'
                if (mDoorOpened.getValue() == true) {
                    newState = State.DOOR_OPEN;
                } 
                else {
                    newState = state;
                }
                break;
            //#state '5.3'
            case DOOR_OPEN:
                //#state '5.3' actions
                localDoorMotor.set(DoorCommand.STOP); //DoorMotor[hallway, side] = STOP
                countDown = SimTime.subtract(countDown, period); //Decrement CountDown.

                //#transition 'T.5.3'
                if (countDown.isLessThanOrEqual(SimTime.ZERO) && 
                    mCarWeight.getWeight() <= Elevator.MaxCarCapacity) {
                        newState = State.DOOR_CLOSING;
                } 
                else {
                    newState = state;
                }
                break;
            //#state '5.4'
            case DOOR_CLOSING:
                //#state '5.4' actions
                localDoorMotor.set(DoorCommand.CLOSE); //DoorMotor[hallway,side] = CLOSE

                //#transition 'T.5.4'
                if (mDoorClosed.getValue() == true && 
                    mCarWeight.getWeight() <= Elevator.MaxCarCapacity) {
                        newState = State.DOOR_CLOSED;
                }
                //#transition 'T.5.5'
                else if (mDoorOpened.getValue() == false && 
                    mCarWeight.getWeight() > Elevator.MaxCarCapacity) {
                        newState = State.DOOR_OPENING;
                }
                //#transition 'T.5.6'
                else if (mDoorReversal.getEitherReversed() == true && reversedOnce == true &&
                    mCarWeight.getWeight() <= Elevator.MaxCarCapacity) {
                        newState = State.DOOR_NUDGE;
                } 
                //#transition 'T.5.7'
                else if (mDoorReversal.getEitherReversed() == true && reversedOnce == false &&
                    mCarWeight.getWeight() <= Elevator.MaxCarCapacity) {
                        newState = State.DOOR_REVERSED;
                }
                else { 
                    newState = state;
                }
                break;
            //#state '5.5'
            case DOOR_REVERSED:
                //#state '5.5' actions
                reversedOnce = true;
                //#transition 'T.5.8'
                newState = State.DOOR_OPENING;
                break;
            //#state '5.6'
            case DOOR_NUDGE:
                //#state '5.6' actions
                localDoorMotor.set(DoorCommand.NUDGE); //DoorMotor[hallway,side] = NUDGE

                //#transition 'T.5.9'
                if(mDoorClosed.getValue() == true && 
                    mCarWeight.getWeight() <= Elevator.MaxCarCapacity) {
                        newState = State.DOOR_CLOSED;
                }
                //#transition 'T.5.10'
                else if(mDoorOpened.getValue() == false && 
                    mCarWeight.getWeight() > Elevator.MaxCarCapacity) {
                        newState = State.DOOR_OPENING;
                }
                else {
                    newState = state;
                }
                break;
            default:
                throw new RuntimeException("State " + state + " was not recognized.");
        }
        
        //log the results of this iteration
        if (state == newState) {
            log("remains in state: ",state);
        } else {
            log("Transition:",state,"->",newState);
        }

        //update the state variable
        state = newState;

        //report the current state
        setState(STATE_KEY,newState.toString());

        //Schedule the next iteration of the controller
        //You must do this at the end of the timer callback 
        //in order to restart the timer.
        timer.start(period);
    }
}
