/**
 * Since this example focuses on Motion Control, lets print everything related to MP in a clean 
 * format.  Expect to see something like......
 * 
 * Hold            2048            0               0               1                                                                               5.0             0.0
 * Hold            2048            0               0               1                                                                               5.0             0.0
 * outputEnable    topBufferRem    topBufferCnt    btmBufferCnt    IsValid     HasUnderrun      IsUnderrun          IsLast         VelOnly         targPos         targVel
 * Hold            2048            0               0               1                                                                               5.0             0.0
 * Hold            2048            0               0               1                                                                               5.0             0.0
 * Hold            2048            0               0               1                                                                               5.0             0.0
 * Hold            2048            0               0               1                                                                               5.0             0.0
 * Hold            2048            0               0               1                                                                               5.0             0.0
 * Hold            2048            0               0               1                                                                               5.0             0.0
 * Hold            2048            0               0               1                                                                               5.0             0.0
 * outputEnable    topBufferRem    topBufferCnt    btmBufferCnt    IsValid     HasUnderrun      IsUnderrun          IsLast         VelOnly         targPos         targVel
 * Hold            2048            0               0               1                                                                               5.0             0.0
 * Hold            2048            0               0               1                                                                               5.0             0.0
 * Hold            2048            0               0               1                                                                               5.0             0.0
 * 
 * ...where the columns are reprinted occasionally so you know whats up.
 */
package frc.robot.subsystems.utils;

import com.ctre.phoenix.motion.*;

public class Instrumentation {

	static double timeout = 0;
	static int count = 0;

	private static final String[] _table = {" Dis ", " En  ", "Hold "};

	public static void OnUnderrun() {
		//system..out.format("%s\n", "UNDERRUN");
	}

	public static void OnNoProgress() {
		//system..out.format("%s\n", "NOPROGRESS");
	}

	static private String StrOutputEnable(SetValueMotionProfile sv) {
		/* convert sv to string equiv */
		if (sv == null)
			return "null";
		if (sv.value > 3)
			return "Inval";
		return _table[sv.value];
	}

	public static void process(MotionProfileStatus status, double pos,
			double vel, double heading) {
		double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();

		if ((now - timeout) > 0.2) {
			timeout = now;
			/* fire a loop every 200ms */

			if (--count <= 0) {
				count = 8;
				/* every 8 loops, print our columns */

				//system..out.format("%-9s\t", "outEn");
				//system..out.format("%-9s\t", "topCnt");
				//system..out.format("%-9s\t", "topRem");
				//system..out.format("%-9s\t", "btmCnt");
				//system..out.format("%-9s\t", "IsValid");
				//system..out.format("%-9s\t", "HasUnder");
				//system..out.format("%-9s\t", "IsUnder");
				//system..out.format("%-9s\t", "IsLast");
				//system..out.format("%-9s\t", "targPos");
				//system..out.format("%-9s\t", "targVel");
				//system..out.format("%-9s\t", "SlotSel0");
				//system..out.format("%-9s\t", "timeDurMs");

				//system..out.format("\n");
			}
			/* every loop, print our values */
			//system..out.format("%-9s\t", StrOutputEnable(status.outputEnable));
			//system..out.format("%-9s\t", status.topBufferCnt);
			//system..out.format("%-9s\t", status.topBufferRem);
			//system..out.format("%-9s\t", status.btmBufferCnt);
			//system..out.format("%-9s\t", (status.activePointValid ? "1" : ""));
			//system..out.format("%-9s\t", (status.hasUnderrun ? "1" : ""));
			//system..out.format("%-9s\t", (status.isUnderrun ? "1" : ""));
			//system..out.format("%-9s\t", (status.isLast ? "1" : ""));
			//system..out.format("%-9s\t", pos);
			//system..out.format("%-9s\t", vel);
			//system..out.format("%-9s\t", status.profileSlotSelect);
			//system..out.format("%-9s\t", status.timeDurMs);

			//system..out.format("\n");
		}
	}
}
