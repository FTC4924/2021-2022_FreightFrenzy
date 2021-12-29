package org.firstinspires.ftc.teamcode.Commands;

import static org.firstinspires.ftc.teamcode.Constants.CommandType.ARM_EXTEND;
import static org.firstinspires.ftc.teamcode.Constants.MAX_ARM_EXTENSION;

/**
 * Command for extending/retracting the arm to a percent of it's limit.
 */
public class ArmExtend extends Command {
    public ArmExtend(double percent) {
        super(ARM_EXTEND);
        this.position = (int)(percent * MAX_ARM_EXTENSION);
    }
}
