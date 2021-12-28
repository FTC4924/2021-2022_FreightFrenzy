package org.firstinspires.ftc.teamcode.Commands;
import org.firstinspires.ftc.teamcode.Commands.Command;

import static org.firstinspires.ftc.teamcode.Constants.CommandType.MOVE;

public class Move extends Command {
    public Move(double distance, double angle, double power) {
        super(MOVE);
        this.distance = distance;
        this.angle = Math.toRadians(angle);
        this.power = power;
    }
}
