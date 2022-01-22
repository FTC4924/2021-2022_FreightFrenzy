package org.firstinspires.ftc.teamcode.Commands;

import androidx.annotation.FloatRange;

import org.firstinspires.ftc.teamcode.AutoBase;

public class Move extends Command {
    public Move(double distance, double angle, @FloatRange(from=0.0, to=1.0) double power) {
        this.distance = distance;
        this.angle = Math.toRadians(angle);
        this.power = power;
    }

    public Move(double distance, @FloatRange(from=0.0, to=1.0) double power) {
        this.distance = distance;
        this.angle = AutoBase.getTargetAngle();
        this.power = power;
    }
}
