/**
* 18649 Fall 2013
* Team 18
* Yi Huo (yhuo)
* Mustafa Bilgen (mbilgen)
* Sairam Krishnan (sbkrishn)
* Abhijit Lele (alele)
*/

package simulator.elevatorcontrol;

import java.util.BitSet;
import simulator.elevatormodules.*;
import simulator.framework.DoorCommand;
import simulator.framework.Hallway;
import simulator.framework.ReplicationComputer;
import simulator.framework.Side;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;
import simulator.payloads.translators.CanPayloadTranslator;


/**
 * Can payload translator for the mDoorMotor message, for a
 * specified hallway and side.
 * @author Sairam Krishnan (sbkrishn)
 */
public class DoorMotorCanPayloadTranslator extends CanPayloadTranslator {

    String name;

    /**
     * CAN payload translator for door closed network messages
     * @param p  CAN payload object whose message is interpreted by this translator
     * @param hallway  replication index
     * @param side  replication index
     */
    public DoorMotorCanPayloadTranslator(WriteableCanMailbox p, Hallway hallway, Side side) {
        super(p,1, MessageDictionary.DOOR_MOTOR_COMMAND_BASE_CAN_ID + 
            ReplicationComputer.computeReplicationId(hallway, side));
        this.name = "DoorOpenedSensor" + ReplicationComputer.makeReplicationString(hallway, side);
    }

    /**
     * CAN payload translator for door closed network messages
     * @param p  CAN payload object whose message is interpreted by this translator
     * @param hallway  replication index
     * @param side  replication index
     */
    public DoorMotorCanPayloadTranslator(ReadableCanMailbox p, Hallway hallway, Side side) {
        super(p,1, MessageDictionary.DOOR_MOTOR_COMMAND_BASE_CAN_ID + 
            ReplicationComputer.computeReplicationId(hallway, side));
        this.name = "DoorOpenedSensor" + ReplicationComputer.makeReplicationString(hallway, side);
    }

    public void setDoorCommand(DoorCommand command) {
        BitSet b = getMessagePayload();
        addIntToBitset(b, command.ordinal(), 0, 32);
        setMessagePayload(b, getByteSize());
    }

    public DoorCommand getDoorCommand() {
        int val = getIntFromBitset(getMessagePayload(), 0, 32);
        for (DoorCommand d : DoorCommand.values()) {
            if (d.ordinal() == val) {
                return d;
            }
        }
        throw new RuntimeException("Unrecognized Direction Value " + val);
    }

    @Override
    public String payloadToString() {
        return name;
    }
}