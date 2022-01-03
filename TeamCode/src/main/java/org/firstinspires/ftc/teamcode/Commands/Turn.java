package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.teamcode.Command;

public class Turn extends Command {
    public Turn(double angle) {
        this.angle = Math.toRadians(angle);
    }
}
