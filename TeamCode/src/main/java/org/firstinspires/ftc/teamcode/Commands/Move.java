package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.teamcode.Command;

public class Move extends Command {
    public Move(double distance, double angle, double power) {
        this.distance = distance;
        this.angle = Math.toRadians(angle);
        this.power = power;
    }
}
