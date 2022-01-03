package org.firstinspires.ftc.teamcode.Commands;

import static org.firstinspires.ftc.teamcode.Constants.MAX_ARM_ROTATION;

public class ArmRotate extends Command {
    public ArmRotate(double percent) {
        this.position = (int)(percent * MAX_ARM_ROTATION);
    }
}
