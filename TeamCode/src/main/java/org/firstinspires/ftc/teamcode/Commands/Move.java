package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.teamcode.AutoBase;

public class Move extends Command {
    public Move(double distance, double angle, double power) {
        this.distance = distance;
        this.angle = Math.toRadians(angle);
        this.power = power;
    }

    public Move(double distance, double power) {
        this.distance = distance;
        this.angle = AutoBase.getTargetAngle();
        this.power = power;
    }
}
