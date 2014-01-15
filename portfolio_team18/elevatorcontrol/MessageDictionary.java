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
import jSimPack.SimTime.SimTimeUnit;

/**
 * This class defines constants for CAN IDs that are used throughout the simulator.
 *
 * The default values will work for early projects.  Later on, you will modify these
 * values when you create a network schedule.
 *
 * @author justinr2
 */
public class MessageDictionary {

    //controller periods
    public final static int NONE = -1;
    public final static SimTime HALL_BUTTON_CONTROL_PERIOD = new SimTime(100, SimTimeUnit.MILLISECOND);
    public final static SimTime CAR_BUTTON_CONTROL_PERIOD = new SimTime(100, SimTimeUnit.MILLISECOND);
    public final static SimTime LANTERN_CONTROL_PERIOD = new SimTime(200, SimTimeUnit.MILLISECOND);
    public final static SimTime CAR_POSITION_CONTROL_PERIOD = new SimTime(50, SimTimeUnit.MILLISECOND);
    public final static SimTime DISPATCHER_PERIOD = new SimTime(50, SimTimeUnit.MILLISECOND);
    public final static SimTime DOOR_CONTROL_PERIOD = new SimTime(10, SimTimeUnit.MILLISECOND);
    public final static SimTime DRIVE_CONTROL_PERIOD = new SimTime(5, SimTimeUnit.MILLISECOND);

    //controller message IDs
    public final static int DRIVE_SPEED_CAN_ID =                0x08648400;
    // NOT USED:
    public final static int DRIVE_COMMAND_CAN_ID =              0x08968400;
    public final static int DESIRED_DWELL_BASE_CAN_ID =         0x09C28800;
    public final static int DESIRED_FLOOR_CAN_ID =              0x09F48800;
    // NOT USED:
    public final static int CAR_POSITION_CAN_ID =               0x09908600;
    // NOT USED:
    public final static int DOOR_MOTOR_COMMAND_BASE_CAN_ID =    0x08328200;
    public final static int HALL_CALL_BASE_CAN_ID =             0x0AEE8C00;
    // NOT USED:
    public final static int HALL_LIGHT_BASE_CAN_ID =            0x0B208C00;
    public final static int CAR_CALL_BASE_CAN_ID =              0x0A588A00;
    // NOT USED:
    public final static int CAR_LIGHT_BASE_CAN_ID =             0x0A8A8A00;
    // NOT USED:
    public final static int CAR_LANTERN_BASE_CAN_ID =           0x0B528E00;
    
    //module message IDs
    public final static int AT_FLOOR_BASE_CAN_ID =              0x092C0600;
    public final static int CAR_LEVEL_POSITION_CAN_ID =         0x095E0800;
    public final static int CAR_WEIGHT_CAN_ID =                 0x0B840E00;
    public final static int CAR_WEIGHT_ALARM_CAN_ID =           0x0BB60E00;
    public final static int DOOR_OPEN_SENSOR_BASE_CAN_ID =      0x0BE81000;
    public final static int DOOR_CLOSED_SENSOR_BASE_CAN_ID =    0x0ABC0C00;
    public final static int DOOR_REVERSAL_SENSOR_BASE_CAN_ID =  0x08C80200;
    public final static int HOISTWAY_LIMIT_BASE_CAN_ID =        0x0C1A1200;
    public final static int EMERGENCY_BRAKE_CAN_ID =            0x0A260A00;
    public final static int LEVELING_BASE_CAN_ID =              0x08FA0400;
    
}
