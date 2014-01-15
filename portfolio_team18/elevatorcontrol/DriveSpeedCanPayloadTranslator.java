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
import simulator.elevatormodules.DriveObject;
import simulator.framework.Direction;
import simulator.framework.Speed;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;
import simulator.payloads.translators.CanPayloadTranslator;

/**
 * Can payload translator for the mDriveSpeed message, which includes an
 * integer speed value and a direction.
 * @author Sairam Krishnan (sbkrishn)
 */
public class DriveSpeedCanPayloadTranslator extends CanPayloadTranslator {

    public DriveSpeedCanPayloadTranslator(WriteableCanMailbox p) {
        super(p, 8, MessageDictionary.DRIVE_SPEED_CAN_ID);
    }
    
    public DriveSpeedCanPayloadTranslator(ReadableCanMailbox p) {
        super(p, 8, MessageDictionary.DRIVE_SPEED_CAN_ID);
    }

    /**
     * This method is required for setting values by reflection in the
     * MessageInjector.  The order of parameters in .mf files should match the
     * signature of this method.
     * All translators must have a set() method with the signature that contains
     * all the parameter values.
     *
     * @param speed
     * @param dir
     */
    public void set(double speed, Direction dir) {
        setSpeed(speed);
        setDirection(dir);
    }
    
    public void setSpeed(double speed) {
        BitSet b = getMessagePayload();
        int bits = Float.floatToIntBits((float)speed);
        addIntToBitset(b, bits, 0, 32);
        setMessagePayload(b, getByteSize());
    }

    public double getSpeed() {
        int bits = getIntFromBitset(getMessagePayload(), 0, 32);
        double speed = (double)Float.intBitsToFloat(bits);
        return speed;
    }

    public void setDirection(Direction dir) {
        BitSet b = getMessagePayload();
        addIntToBitset(b, dir.ordinal(), 48, 32);
        setMessagePayload(b, getByteSize());
    }

    public Direction getDirection() {
        int val = getIntFromBitset(getMessagePayload(), 48, 32);
        for (Direction d : Direction.values()) {
            if (d.ordinal() == val) {
                return d;
            }
        }
        throw new RuntimeException("Unrecognized Direction Value " + val);
    }

    @Override
    public String payloadToString() {
        return "DriveSpeedCommand:  speed=" + getSpeed() + " direction=" + getDirection();
    }
}
