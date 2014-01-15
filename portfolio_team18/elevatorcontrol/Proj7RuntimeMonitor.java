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
import simulator.elevatorcontrol.DesiredFloorCanPayloadTranslator;
import simulator.framework.DoorCommand;
import simulator.framework.Elevator;
import simulator.framework.Hallway;
import simulator.framework.Direction;
import simulator.framework.Harness;
import simulator.framework.RuntimeMonitor;
import simulator.framework.Side;
import simulator.payloads.AtFloorPayload.ReadableAtFloorPayload;
import simulator.payloads.CarWeightPayload.ReadableCarWeightPayload;
import simulator.payloads.DoorClosedPayload.ReadableDoorClosedPayload;
import simulator.payloads.DoorMotorPayload.ReadableDoorMotorPayload;
import simulator.payloads.DoorOpenPayload.ReadableDoorOpenPayload;
import simulator.payloads.DoorReversalPayload.ReadableDoorReversalPayload;
import simulator.payloads.HallLightPayload.ReadableHallLightPayload;

/**
 * This monitoring class gives you a starting point for the performance checks
 * you will do in project 7.
 *
 * This monitor detects the number of stops where the car becomes overweight
 * when the door are open.  Each stop counted at most once, unless the doors
 * close completely and then reopen at the same floor.
 *
 * See the documentation of simulator.framework.RuntimeMonitor for more details.
 * 
 * @author Justin Ray
 */
public class Proj7RuntimeMonitor extends RuntimeMonitor {

    private final int NO_FLOOR = -1;

    private DoorStateMachine doorState = new DoorStateMachine();
    private WeightStateMachine weightState = new WeightStateMachine();
    private boolean wasOverweight = false;
    private boolean reversing = false;
    private int currentFloor = NO_FLOOR;

    private Stopwatch reversalSW = new Stopwatch();

    private int wastedOpenings = 0;
    private SimTime doorReversalTime = SimTime.ZERO;
    private int overWeightCount = 0;

    private boolean[][] atFloorArray = new boolean[Elevator.numFloors][2];

    public Proj7RuntimeMonitor() {
        super();
    }

    private int getCurrentFloor() {
        for (int i = 0; i < Elevator.numFloors; i++) {
            if (atFloorArray[i][0] || atFloorArray[i][1])
                return i + 1;
        }
        return NO_FLOOR;
    }

    @Override
    protected String[] summarize() {
        String[] arr = new String[3];
        arr[0] = "Overweight Count: " + overWeightCount;
        arr[1] = "Wasted openings count: " + wastedOpenings;
        arr[2] = "Time spent on door reversals: " + doorReversalTime;
        return arr;
    }

    public void timerExpired(Object callbackData) {
        //do nothing
    }

    /**************************************************************************
     * high level event methods
     *
     * these are called by the logic in the message receiving methods and the
     * state machines
     **************************************************************************/
    /**
     * Called once when the door starts opening
     * @param hallway which door the event pertains to
     */
    private void doorOpening(Hallway hallway) {
        if (currentFloor == NO_FLOOR) {
            // This shouldn't happen under normal conditions
            // Still, that is a wasted opening since nobody made a call
            // in-between floors
            wastedOpenings++;
            warning("Door at hallway " + hallway + " opened while not at floor");
            return;
        }
    
        Direction dir = mDesiredFloor.getDirection();

        // IF: the hallLight in the direction we're going in is lit
        // OR if the carLight for that floor/hallway is lit, 
        // THEN this opening is OK. else, it's wasted.
        if (!
             (((dir == Direction.UP || dir == Direction.DOWN) &&
                hallLights[currentFloor - 1][hallway.ordinal()][dir.ordinal()].lighted())
             || carLights[currentFloor - 1][hallway.ordinal()].lighted()))
            wastedOpenings++;
    }

    /**
     * Called once when the door starts closing
     * @param hallway which door the event pertains to
     */
    private void doorClosing(Hallway hallway) {
    }

    /**
     * Called once if the door starts opening after it started closing but before
     * it was fully closed.
     * @param hallway which door the event pertains to
     */
    private void doorReopening(Hallway hallway) {
    }

    /**
     * Called once when the doors close completely
     * @param hallway which door the event pertains to
     */
    private void doorClosed(Hallway hallway) {
        //once all doors are closed, check to see if the car was overweight
        if (!doorState.anyDoorOpen()) {
            if (wasOverweight) {
                message("Overweight");
                overWeightCount++;
                wasOverweight = false;
            }
            if (reversing) {
                reversalSW.stop();
                reversing = false;
                doorReversalTime = reversalSW.getAccumulatedTime();
            }
        }
    }

    /**
     * Called once when the doors are fully open
     * @param hallway which door the event pertains to
     */
    private void doorOpened(Hallway hallway) {
        //System.out.println(hallway.toString() + " Door Opened");
    }

    /**
     * Called when the car weight changes
     * @param newWeight new weight of the car
     */
    private void weightChanged(int newWeight) {
        if (newWeight > Elevator.MaxCarCapacity) {
            wasOverweight = true;
        }
    }

    /**************************************************************************
     * low level message receiving methods
     * 
     * These mostly forward messages to the appropriate state machines
     **************************************************************************/
    @Override
    public void receive(ReadableDoorClosedPayload msg) {
        doorState.receive(msg);
    }

    @Override
    public void receive(ReadableDoorOpenPayload msg) {
        doorState.receive(msg);
    }

    @Override
    public void receive(ReadableDoorMotorPayload msg) {
        doorState.receive(msg);
    }

    @Override
    public void receive(ReadableDoorReversalPayload msg) {
        if (reversing)
            return;

        if (msg.isReversing())
        {
            reversalSW.start();
            reversing = true;
        }
    }

    @Override
    public void receive(ReadableCarWeightPayload msg) {
        weightState.receive(msg);
    }

    @Override
    public void receive(ReadableAtFloorPayload msg) {
        atFloorArray[msg.getFloor() - 1][msg.getHallway().ordinal()] = msg.getValue();
        currentFloor = getCurrentFloor();
    }

    private static enum DoorState {
        CLOSED,
        OPENING,
        OPEN,
        CLOSING
    }

    /**
     * Utility class to detect weight changes
     */
    private class WeightStateMachine {

        int oldWeight = 0;

        public void receive(ReadableCarWeightPayload msg) {
            if (oldWeight != msg.weight()) {
                weightChanged(msg.weight());
            }
            oldWeight = msg.weight();
        }
    }

    /**
     * Utility class for keeping track of the door state.
     * 
     * Also provides external methods that can be queried to determine the
     * current door state.
     */
    private class DoorStateMachine {

        DoorState state[] = new DoorState[2];

        public DoorStateMachine() {
            state[Hallway.FRONT.ordinal()] = DoorState.CLOSED;
            state[Hallway.BACK.ordinal()] = DoorState.CLOSED;
        }

        public void receive(ReadableDoorClosedPayload msg) {
            updateState(msg.getHallway());
        }

        public void receive(ReadableDoorOpenPayload msg) {
            updateState(msg.getHallway());
        }

        public void receive(ReadableDoorMotorPayload msg) {
            updateState(msg.getHallway());
        }

        private void updateState(Hallway h) {
            DoorState previousState = state[h.ordinal()];

            DoorState newState = previousState;

            if (allDoorsClosed(h) && allDoorMotorsStopped(h)) {
                newState = DoorState.CLOSED;
            } else if (allDoorsCompletelyOpen(h) && allDoorMotorsStopped(h)) {
                newState = DoorState.OPEN;
                //} else if (anyDoorMotorClosing(h) && anyDoorOpen(h)) {
            } else if (anyDoorMotorClosing(h)) {
                newState = DoorState.CLOSING;
            } else if (anyDoorMotorOpening(h)) {
                newState = DoorState.OPENING;
            }

            if (newState != previousState) {
                switch (newState) {
                    case CLOSED:
                        doorClosed(h);
                        break;
                    case OPEN:
                        doorOpened(h);
                        break;
                    case OPENING:
                        if (previousState == DoorState.CLOSING) {
                            doorReopening(h);
                        } else {
                            doorOpening(h);
                        }
                        break;
                    case CLOSING:
                        doorClosing(h);
                        break;

                }
            }

            //set the newState
            state[h.ordinal()] = newState;
        }

        //door utility methods
        public boolean allDoorsCompletelyOpen(Hallway h) {
            return doorOpeneds[h.ordinal()][Side.LEFT.ordinal()].isOpen()
                    && doorOpeneds[h.ordinal()][Side.RIGHT.ordinal()].isOpen();
        }

        public boolean anyDoorOpen() {
            return anyDoorOpen(Hallway.FRONT) || anyDoorOpen(Hallway.BACK);

        }

        public boolean anyDoorOpen(Hallway h) {
            return !doorCloseds[h.ordinal()][Side.LEFT.ordinal()].isClosed()
                    || !doorCloseds[h.ordinal()][Side.RIGHT.ordinal()].isClosed();
        }

        public boolean allDoorsClosed(Hallway h) {
            return (doorCloseds[h.ordinal()][Side.LEFT.ordinal()].isClosed()
                    && doorCloseds[h.ordinal()][Side.RIGHT.ordinal()].isClosed());
        }

        public boolean allDoorMotorsStopped(Hallway h) {
            return doorMotors[h.ordinal()][Side.LEFT.ordinal()].command() == DoorCommand.STOP && doorMotors[h.ordinal()][Side.RIGHT.ordinal()].command() == DoorCommand.STOP;
        }

        public boolean anyDoorMotorOpening(Hallway h) {
            return doorMotors[h.ordinal()][Side.LEFT.ordinal()].command() == DoorCommand.OPEN || doorMotors[h.ordinal()][Side.RIGHT.ordinal()].command() == DoorCommand.OPEN;
        }

        public boolean anyDoorMotorClosing(Hallway h) {
            return doorMotors[h.ordinal()][Side.LEFT.ordinal()].command() == DoorCommand.CLOSE || doorMotors[h.ordinal()][Side.RIGHT.ordinal()].command() == DoorCommand.CLOSE;
        }
    }

    /**
     * Keep track of time and decide whether to or not to include the last interval
     */
    private class ConditionalStopwatch {

        private boolean isRunning = false;
        private SimTime startTime = null;
        private SimTime accumulatedTime = SimTime.ZERO;

        /**
         * Call to start the stopwatch
         */
        public void start() {
            if (!isRunning) {
                startTime = Harness.getTime();
                isRunning = true;
            }
        }

        /**
         * stop the stopwatch and add the last interval to the accumulated total
         */
        public void commit() {
            if (isRunning) {
                SimTime offset = SimTime.subtract(Harness.getTime(), startTime);
                accumulatedTime = SimTime.add(accumulatedTime, offset);
                startTime = null;
                isRunning = false;
            }
        }

        /**
         * stop the stopwatch and discard the last interval
         */
        public void reset() {
            if (isRunning) {
                startTime = null;
                isRunning = false;
            }
        }

        public SimTime getAccumulatedTime() {
            return accumulatedTime;
        }

        public boolean isRunning() {
            return isRunning;
        }
    }

    /**
     * Keep track of the accumulated time for an event
     */
    private class Stopwatch {

        private boolean isRunning = false;
        private SimTime startTime = null;
        private SimTime accumulatedTime = SimTime.ZERO;

        /**
         * Start the stopwatch
         */
        public void start() {
            if (!isRunning) {
                startTime = Harness.getTime();
                isRunning = true;
            }
        }

        /**
         * Stop the stopwatch and add the interval to the accumulated total
         */
        public void stop() {
            if (isRunning) {
                SimTime offset = SimTime.subtract(Harness.getTime(), startTime);
                accumulatedTime = SimTime.add(accumulatedTime, offset);
                startTime = null;
                isRunning = false;
            }
        }

        public SimTime getAccumulatedTime() {
            return accumulatedTime;
        }

        public boolean isRunning() {
            return isRunning;
        }
    }

}
