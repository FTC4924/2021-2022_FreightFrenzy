package org.firstinspires.ftc.teamcode.Commands;

import static org.firstinspires.ftc.teamcode.Constants.MAX_ARM_EXTENSION;

public class ArmExtend extends Command {
    public ArmExtend(double percent) {
        this.position = (int)(percent * MAX_ARM_EXTENSION);
    }
}
