package org.firstinspires.ftc.teamcode.Commands;
import org.firstinspires.ftc.teamcode.AutoBase;
import org.firstinspires.ftc.teamcode.Commands.Command;

import static org.firstinspires.ftc.teamcode.Constants.CommandType.MOVE;

/**
 * Command that moves the robot for a distance at an angle with a power.
 */
public class Move extends Command {
    public Move(double distance, double angle, double power) {
        super(MOVE);
        this.distance = distance;
        this.angle = Math.toRadians(angle);
        this.power = power;
    }

    public Move(double distance, double power) {
        super(MOVE);
        this.distance = distance;
        this.angle = AutoBase.getTargetAngle();
        this.power = power;
    }
}
