/**
* 18649 Fall 2013
* Team 18
* Yi Huo (yhuo)
* Mustafa Bilgen (mbilgen)
* Sairam Krishnan (sbkrishn)
* Abhijit Lele (alele)
*/
package simulator.elevatorcontrol;

import java.util.HashMap;
import simulator.elevatormodules.AtFloorCanPayloadTranslator;
import simulator.elevatormodules.DoorClosedCanPayloadTranslator;
import simulator.elevatormodules.DoorReversalCanPayloadTranslator;
import simulator.elevatormodules.DriveObject;
import simulator.payloads.CANNetwork;
import simulator.framework.Direction;
import simulator.framework.DoorCommand;
import simulator.framework.Elevator;
import simulator.framework.Hallway;
import simulator.framework.Harness;
import simulator.framework.ReplicationComputer;
import simulator.framework.Side;
import simulator.payloads.CanMailbox;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.translators.BooleanCanPayloadTranslator;

/**
 * This class provides some example utility classes that might be useful in more
 * than one spot. It is okay to create new classes (or modify the ones given
 * below), but you may not use utility classes in such a way that they
 * constitute
 * a communication channel between controllers.
 * 
 * @author justinr2
 */
public class Utility
{

	public static class HallCallArray
	{
		HashMap<Integer, BooleanCanPayloadTranslator> translatorArray = new HashMap<Integer, BooleanCanPayloadTranslator>();
		private int maxFloor;


		public HallCallArray(int maxFloor, CANNetwork.CanConnection conn)
		{
			this.maxFloor = maxFloor;
			for (int f = 1; f <= maxFloor; f++)
			{
				for (Hallway b : Hallway.replicationValues)
				{
					for (Direction d : Direction.replicationValues)
					{
						int idx = ReplicationComputer.computeReplicationId(f, b, d);
						ReadableCanMailbox m = CanMailbox
								.getReadableCanMailbox(MessageDictionary.HALL_CALL_BASE_CAN_ID
										+ idx);
						BooleanCanPayloadTranslator t = new BooleanCanPayloadTranslator(m);
						conn.registerTimeTriggered(m);
						translatorArray.put(idx, t);
					}
				}
			}
		}

		public boolean get(int f, Hallway b, Direction d)
		{
			if (f < 1 || f > maxFloor)
				return false;
			int idx = ReplicationComputer.computeReplicationId(f, b, d);
			return translatorArray.get(idx).getValue();
		}

		public boolean getAnyHallway(int f, Direction d)
		{
			return get(f, Hallway.FRONT, d) || get(f, Hallway.BACK, d);
		}

		public boolean getAny(int f)
		{
			return getAnyHallway(f, Direction.UP) || getAnyHallway(f, Direction.DOWN);
		}
	}

	public static class CarCallArray
	{
		HashMap<Integer, BooleanCanPayloadTranslator> translatorArray = new HashMap<Integer, BooleanCanPayloadTranslator>();
		int maxFloor;


		public CarCallArray(int maxFloor, CANNetwork.CanConnection conn)
		{
			this.maxFloor = maxFloor;
			for (int f = 1; f <= maxFloor; f++)
			{
				for (Hallway b : Hallway.replicationValues)
				{
					int idx = ReplicationComputer.computeReplicationId(f, b);
					ReadableCanMailbox m = CanMailbox
							.getReadableCanMailbox(MessageDictionary.CAR_CALL_BASE_CAN_ID + idx);
					BooleanCanPayloadTranslator t = new BooleanCanPayloadTranslator(m);
					conn.registerTimeTriggered(m);
					translatorArray.put(idx, t);
				}
			}
		}

		public boolean get(int f, Hallway b)
		{
			if (f < 1 || f > maxFloor)
				return false;
			int idx = ReplicationComputer.computeReplicationId(f, b);
			return translatorArray.get(idx).getValue();
		}

		public boolean getAny(int f)
		{
			return get(f, Hallway.FRONT) || get(f, Hallway.BACK);
		}
	}

	public static class DoorClosedArray
	{

		HashMap<Integer, DoorClosedCanPayloadTranslator> translatorArray = new HashMap<Integer, DoorClosedCanPayloadTranslator>();
		public final Hallway hallway;


		public DoorClosedArray(Hallway hallway, CANNetwork.CanConnection conn)
		{
			this.hallway = hallway;
			for (Side s : Side.values())
			{
				int index = ReplicationComputer.computeReplicationId(hallway, s);
				ReadableCanMailbox m = CanMailbox
						.getReadableCanMailbox(MessageDictionary.DOOR_CLOSED_SENSOR_BASE_CAN_ID
								+ index);
				DoorClosedCanPayloadTranslator t = new DoorClosedCanPayloadTranslator(m, hallway, s);
				conn.registerTimeTriggered(m);
				translatorArray.put(index, t);
			}
		}

		public boolean getBothClosed()
		{
			return translatorArray
					.get(ReplicationComputer.computeReplicationId(hallway, Side.LEFT)).getValue()
					&& translatorArray.get(
							ReplicationComputer.computeReplicationId(hallway, Side.RIGHT))
							.getValue();
		}
	}

	public static class DoorReversalArray
	{

		HashMap<Integer, DoorReversalCanPayloadTranslator> translatorArray = new HashMap<Integer, DoorReversalCanPayloadTranslator>();
		public final Hallway hallway;


		public DoorReversalArray(Hallway hallway, CANNetwork.CanConnection conn)
		{
			this.hallway = hallway;
			for (Side s : Side.values())
			{
				int index = ReplicationComputer.computeReplicationId(hallway, s);
				ReadableCanMailbox m = CanMailbox
						.getReadableCanMailbox(MessageDictionary.DOOR_REVERSAL_SENSOR_BASE_CAN_ID
								+ index);
				DoorReversalCanPayloadTranslator t = new DoorReversalCanPayloadTranslator(m, hallway, s);
				conn.registerTimeTriggered(m);
				translatorArray.put(index, t);
			}
		}

		public boolean getEitherReversed()
		{
			return translatorArray.get(ReplicationComputer.computeReplicationId(hallway, Side.LEFT)).getValue()
					|| translatorArray.get(ReplicationComputer.computeReplicationId(hallway, Side.RIGHT)).getValue();
		}
	}

	public static class AtFloorArray
	{

		public HashMap<Integer, AtFloorCanPayloadTranslator> networkAtFloorsTranslators = new HashMap<Integer, AtFloorCanPayloadTranslator>();
		public final int numFloors = Elevator.numFloors;


		public AtFloorArray(CANNetwork.CanConnection conn)
		{
			for (int i = 0; i < numFloors; i++)
			{
				int floor = i + 1;
				for (Hallway h : Hallway.replicationValues)
				{
					int index = ReplicationComputer.computeReplicationId(floor, h);
					ReadableCanMailbox m = CanMailbox
							.getReadableCanMailbox(MessageDictionary.AT_FLOOR_BASE_CAN_ID + index);
					AtFloorCanPayloadTranslator t = new AtFloorCanPayloadTranslator(m, floor, h);
					conn.registerTimeTriggered(m);
					networkAtFloorsTranslators.put(index, t);
				}
			}
		}

		public boolean isAtFloor(int floor, Hallway hallway)
		{
			return networkAtFloorsTranslators.get(
					ReplicationComputer.computeReplicationId(floor, hallway)).getValue();
		}

		public int getCurrentFloor()
		{
			int retval = MessageDictionary.NONE;
			for (int i = 0; i < numFloors; i++)
			{
				int floor = i + 1;
				for (Hallway h : Hallway.replicationValues)
				{
					int index = ReplicationComputer.computeReplicationId(floor, h);
					AtFloorCanPayloadTranslator t = networkAtFloorsTranslators.get(index);
					if (t.getValue())
					{
						if (retval == MessageDictionary.NONE)
						{
							//this is the first true atFloor
							retval = floor;
						}
						else if (retval != floor)
						{
							//found a second floor that is different from the first one
							throw new RuntimeException(
									"AtFloor is true for more than one floor at "
											+ Harness.getTime());
						}
					}
				}
			}
			return retval;
		}
	}

    // Commit point slack - in mm.
    public static final int CP_SLACK = 500;

	/**
	 * @brief Checks whether the commit point is reached for the given floor
	 * 
	 * @param pos Current position, in mm
	 * @param speed Current speed, in m/s
	 * @param dir Current direction
	 * @param floor The floor to check against
	 * 
	 * @return true if floor is within stopping distance, else false
	 */
	public static boolean getCommitPoint(int pos, double speed, Direction dir, int floor)
	{
		double target = (floor - 1) * Elevator.DISTANCE_BETWEEN_FLOORS * 1000;
		double a = DriveObject.Deceleration * 1000;
        speed = speed * 1000;

		// Vf^2 = Vo^2 + 2 * a * x
		// If we slam the brakes now, we will travel v^2 / (2*a meters to stop.
		// The distance to floor must be greater than that.
		double distanceToStop = speed * speed / (2 * a);

        int slack = CP_SLACK;
        if (speed < DriveObject.SlowSpeed)
            slack = 0;

		// Since the CarPosition can be off by 10 cm, add another 10 cm
		// in the direction we're in for some slack + network delays.
		if (dir == Direction.UP) {
			pos += slack;
			if (pos + distanceToStop > target)
				return false;
			else
				return true;
		}
		else if (dir == Direction.DOWN) {
			pos -= slack;
			if (pos - distanceToStop < target)
				return false;
			else
				return true;
		}
		else if (dir == Direction.STOP) {
			return true;
		}
		else
			throw new IllegalArgumentException("Invalid direction: " + dir);

	}

	/**
	 * @brief Checks whether the elevator has reached the commit point for a floor.
	 * Does not tell you if the elevator can actually stop at that floor b/c drive speed
	 * is not a parameter.
	 *
	 * @param pos Current position, in mm
	 * @param dir Current direction
	 * @param floor The floor to check against
	 * 
	 * @return true if floor is within stopping distance, else false
	 */
	public static boolean atCommitPoint(int pos, Direction dir, int floor)
	{
		double target = (floor - 1) * Elevator.DISTANCE_BETWEEN_FLOORS * 1000;

		// Since the CarPosition can be off by 10 cm, add another 10 cm
		// in the direction we're in for some slack + network delays.
		if (dir == Direction.UP) {
			return pos <= target && pos >= target - CP_SLACK;
		}
		else if (dir == Direction.DOWN) {
			return pos >= target && pos <= target + CP_SLACK;
		}
		else if (dir == Direction.STOP) {
			return true;
		}
		else {
			throw new IllegalArgumentException("Invalid direction: " + dir);
		}
	}
}
