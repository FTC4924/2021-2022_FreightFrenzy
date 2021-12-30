package org.firstinspires.ftc.teamcode.Commands;

import static org.firstinspires.ftc.teamcode.Constants.CommandType.ARM_ROTATE;
import static org.firstinspires.ftc.teamcode.Constants.MAX_ARM_ROTATION;

/**
 * Command for raising/lowering the arm to a percent of it's limit.
 */
public class ArmRotate extends Command {
    public ArmRotate(double percent) {
        super(ARM_ROTATE);
        this.position = (int)(percent * MAX_ARM_ROTATION);
    }
}
