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
import simulator.elevatorcontrol.Utility.AtFloorArray;
import simulator.elevatorcontrol.Utility.DoorClosedArray;
import simulator.elevatorcontrol.DesiredFloorCanPayloadTranslator;
import simulator.elevatormodules.DoorClosedCanPayloadTranslator;
import simulator.framework.Controller;
import simulator.framework.Direction;
import simulator.framework.Hallway;
import simulator.framework.ReplicationComputer;
import simulator.framework.Side;
import simulator.payloads.CanMailbox;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;
import simulator.payloads.HallCallPayload;
import simulator.payloads.HallCallPayload.ReadableHallCallPayload;
import simulator.payloads.HallLightPayload;
import simulator.payloads.HallLightPayload.WriteableHallLightPayload;
import simulator.payloads.translators.BooleanCanPayloadTranslator;

/**
 * HallButtonControl Implementation
 *
 * Input interface:
 * - mAtFloor[f, b]
 * - mDesiredFloor
 * - mDoorClosed[b, r]
 * - HallCall[f, b, d]
 *
 * Output interace:
 * - HallLight[f, b, d]
 * - mHallCall[f, b, d]
 *
 * @author mbilgen
 */

public class HallButtonControl extends Controller {

    // DECLARATIONS

    //Local physical state
    private ReadableHallCallPayload localHallCall;
    private WriteableHallLightPayload localHallLight;

    //Network interface
    private AtFloorArray atFloor;

    private DoorClosedArray doorClosed;

    private ReadableCanMailbox networkDesiredFloor;
    private DesiredFloorCanPayloadTranslator mDesiredFloor;

    private WriteableCanMailbox networkHallCall;
    private BooleanCanPayloadTranslator mHallCall;

    //Instance fields
    private final Hallway hallway;
    private final Direction direction;
    private final int floor;

    //Time
    private SimTime period;

    //State enumerations
    private enum State {
        STATE_OFF,
        STATE_ON,
    }

    private State state = State.STATE_OFF;

    //State variables

    private int currentFloor;

    /**
     * @param floor Which floor this controller is at
     * @param hallway Which hallway this controller is at
     * @param direction Which direction this controller points to
     * @param period How often should this controller run
     * @param verbose Whether this controller should spam outputs
     */
    public HallButtonControl(int floor, Hallway hallway, Direction direction, 
            SimTime period, boolean verbose) {
        // Call superclass constructor
        super("HallButtonControl" + ReplicationComputer.makeReplicationString(
                    floor, hallway, direction), verbose);

        log("Created HallButtonControl",
                ReplicationComputer.makeReplicationString(floor, hallway,
                direction),
                " with period = ",
                period);

        this.floor = floor;
        this.hallway = hallway;
        this.direction = direction;
        this.period = period;

        //Initialize physical state
        
        localHallCall = HallCallPayload.getReadablePayload(floor, hallway, 
                direction);
        physicalInterface.registerTimeTriggered(localHallCall);

        localHallLight = HallLightPayload.getWriteablePayload(floor,
                hallway, direction);
        physicalInterface.sendTimeTriggered(localHallLight, period);

        //Initialize network interface

        networkHallCall = CanMailbox.getWriteableCanMailbox(
                MessageDictionary.HALL_CALL_BASE_CAN_ID
                + ReplicationComputer.computeReplicationId(floor, hallway, 
                    direction));
        mHallCall = new BooleanCanPayloadTranslator(networkHallCall);
        canInterface.sendTimeTriggered(networkHallCall, period);

        networkDesiredFloor = CanMailbox.getReadableCanMailbox(
                MessageDictionary.DESIRED_FLOOR_CAN_ID);
        mDesiredFloor = new 
            DesiredFloorCanPayloadTranslator(networkDesiredFloor); 
        canInterface.registerTimeTriggered(networkDesiredFloor);

        doorClosed = new DoorClosedArray(hallway, canInterface);

        atFloor = new AtFloorArray(canInterface);

        //Start controller
        timer.start(period);
    }

    public void timerExpired(Object callbackData) {
        State newState = state;
        switch (state) {
            case STATE_OFF:
                //State actions for OFF
                localHallLight.set(false);
                mHallCall.set(false);
                currentFloor = atFloor.getCurrentFloor();

                //Transition 'T.8.1'
                if (localHallCall.pressed()) {
                    newState = State.STATE_ON;
                }
                else {
                    newState = state;
                }
                break;
            case STATE_ON:
                //State actions for ON
                localHallLight.set(true);
                mHallCall.set(true);
                currentFloor = atFloor.getCurrentFloor();

                //Transition 'T.8.2'
                if (    !localHallCall.pressed()
                     && currentFloor == floor
                     && (mDesiredFloor.getHallway() == hallway
                        || mDesiredFloor.getHallway() == Hallway.BOTH)
                     && mDesiredFloor.getDirection() == direction
                     && (!doorClosed.getBothClosed())) {
                    newState = State.STATE_OFF;
                }
                else {
                    newState = state;
                }
                break;
            default:
                throw new RuntimeException("Invalid State: " + state);

        }
    
        if (state == newState)
            log("remains in state: ", state);
        else
            log("Transition: ", state, "->", newState);

        state = newState;

        setState(STATE_KEY, newState.toString());

        timer.start(period);

    }
}
