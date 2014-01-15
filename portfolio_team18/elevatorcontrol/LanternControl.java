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
import simulator.payloads.CarLanternPayload.WriteableCarLanternPayload;
import simulator.payloads.translators.BooleanCanPayloadTranslator;
import simulator.elevatorcontrol.Utility;
import simulator.elevatorcontrol.Utility.*;

/**
 * LanternControl Implementation
 *
 * Inputs:
 *  mDoorClosed[b, r]
 *  mDesiredFloor
 *  mAtFloor[f, b]
 *
 * Outputs:
 *  CarLantern[d]
 *
 * @author Sairam Krishnan (sbkrishn)
 */
public class LanternControl extends Controller {

    /***************************************************************************
     * Declarations
     **************************************************************************/
    //note that inputs are Readable objects, while outputs are Writeable object

    //local physical state for the CarLantern
    private WriteableCarLanternPayload localCarLantern;

    //DoorClosed message
    private DoorClosedArray mFrontDoorClosed, mBackDoorClosed;
    
    //AtFloor message
    private AtFloorArray mAtFloor; 

    //DesiredFloor message
    private ReadableCanMailbox networkDesiredFloor;
    private DesiredFloorCanPayloadTranslator mDesiredFloor;

    //Keep track of which instance this is.
    private final Direction lanternDirection;

    //store the period for the controller
    private SimTime period;

    //enumerate states
    private enum State {
        LANTERN_OFF,
        LANTERN_ON
    }

    //State variable initialized to the initial state DOOR_CLOSED.
    private State state = State.LANTERN_OFF;

    /**
     * The arguments listed in the .cf configuration file should match the order and
     * type given here.
     *
     * For your elevator controllers, you should make sure that the constructor matches
     * the method signatures in ControllerBuilder.makeAll().
     */
    public LanternControl(Direction dir, SimTime period, boolean verbose) {
        //call to the Controller superclass constructor is required
        super("LanternControl" + ReplicationComputer.makeReplicationString(dir), verbose);
        
        //stored the constructor arguments in internal state
        this.period = period;
        this.lanternDirection = dir;

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
        log("Created LanternControl with period = ", period);
        
        //Based on the value of dir, create a payload for the appropriate CarLantern.
        localCarLantern = CarLanternPayload.getWriteablePayload(dir);
        //Register the payload to be sent periodically -- whatever value is stored
        //in the localDoorMotor objects will be sent out periodically with the period
        //specified by the period parameter.
        physicalInterface.sendTimeTriggered(localCarLantern, period);

        /*
         * Registration for network messages. If we can't use a Utility object for
         * the message;
         *   (a) Create a CanMailbox - this object has the binary representation of the 
         *       message data. 
         *   (b) To register for network messages from the smart sensors or other objects
         *       defined in elevator modules, use the translators already defined in 
         *       elevatormodules package. These translators are specific to one type of message.
         *   (c) Register to receive periodic updates to the mailbox via the CAN network.
         *       The period of updates will be determined by the sender of the message.
         */


        //Registration for AtFloor message (using Utility.AtFloorArray)
        mAtFloor = new AtFloorArray(canInterface);

        //Registration for DoorClosed message (using Utility.DoorClosedArray)
        mFrontDoorClosed = new DoorClosedArray(Hallway.FRONT, canInterface);
        mBackDoorClosed = new DoorClosedArray(Hallway.BACK, canInterface);

        //Registration for the DesiredFloor message
        networkDesiredFloor = CanMailbox.getReadableCanMailbox(MessageDictionary.DESIRED_FLOOR_CAN_ID);
        mDesiredFloor = new DesiredFloorCanPayloadTranslator(networkDesiredFloor);
        canInterface.registerTimeTriggered(networkDesiredFloor);

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
        
        //Compute the DesiredDirection by comparing CurrentFloor
        //with the dispatcher's DesiredFloor.
        Direction desiredDirection = mDesiredFloor.getDirection();
        int currentFloor = mAtFloor.getCurrentFloor();
        int desiredFloor = mDesiredFloor.getFloor();

        switch (state) {
            //#state 1
            case LANTERN_OFF:
                //#state 1 actions
                localCarLantern.set(false); //CarLantern[d] = Off

                //#transition 'T.7.1'
                if (currentFloor == MessageDictionary.NONE) {
                   newState = state;
                }
                else if (((Elevator.hasLanding(currentFloor, Hallway.FRONT) && 
                    mAtFloor.isAtFloor(currentFloor, Hallway.FRONT) &&
                    !mFrontDoorClosed.getBothClosed()) || 
                    (Elevator.hasLanding(currentFloor, Hallway.BACK) && 
                    mAtFloor.isAtFloor(currentFloor, Hallway.BACK) &&
                    !mBackDoorClosed.getBothClosed())) && 
                    (desiredDirection == lanternDirection)) {
                     newState = State.LANTERN_ON;
                }
                else {
                    newState = state;
                }
                break;
            //#state 2
            case LANTERN_ON:
                //#state 2 actions
                localCarLantern.set(true); //CarLantern[d] = On

                //#transition 'T.7.2'
                if (mFrontDoorClosed.getBothClosed() && mBackDoorClosed.getBothClosed()) {
                    newState = State.LANTERN_OFF;
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
