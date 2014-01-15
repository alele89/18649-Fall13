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
import simulator.framework.Hallway;
import simulator.framework.ReplicationComputer;
import simulator.payloads.translators.CanPayloadTranslator;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;

/** 
 * Handle mDesiredDwell message for a specified hallway.
 * @author Sairam Krishnan
 */
public class DesiredDwellCanPayloadTranslator extends CanPayloadTranslator {

    Hallway hallway;
    /**
     * Constructor for WriteableCanMailbox. You should always implement both a 
     * Writeable and Readable constructor so the same translator can be used for
     * both objects
     * @param payload
     */
    public DesiredDwellCanPayloadTranslator(WriteableCanMailbox payload, Hallway hallway) {
        super(payload, 4, MessageDictionary.DESIRED_DWELL_BASE_CAN_ID + 
            ReplicationComputer.computeReplicationId(hallway));
        this.hallway = hallway;
    }

    /**
     * Constructor for ReadableCanMailbox.  You should always implement both a 
     * Writeable and Readable constructor so the same translator can be used for
     * both objects
     * @param payload
     */
    public DesiredDwellCanPayloadTranslator(ReadableCanMailbox payload, Hallway hallway) {
        super(payload, 4, MessageDictionary.DESIRED_DWELL_BASE_CAN_ID +
            ReplicationComputer.computeReplicationId(hallway));
        this.hallway = hallway;
    }
    
    public void set(int value) {
        setValue(value);
    }

    /**
     * This method is required for setting values by reflection in the
     * MessageInjector.  The order of parameters in .mf files should match the
     * signature of this method.
     * All translators must have a set() method with the signature that contains
     * all the parameter values.
     *
     * @param value Dwell time in seconds
     */
    public void setValue(int value) {
        BitSet b = new BitSet();
        addIntToBitset(b, value, 0, 32);
        setMessagePayload(b, getByteSize());
    }

    public long getValue() {
        return getIntFromBitset(getMessagePayload(), 0, 32);
    }

    /**
     * Implement a printing method for the translator.
     * @return
     */
    @Override
    public String payloadToString() {
        return "DesiredDwell[" + hallway + "] = " + getValue();
    }
}
