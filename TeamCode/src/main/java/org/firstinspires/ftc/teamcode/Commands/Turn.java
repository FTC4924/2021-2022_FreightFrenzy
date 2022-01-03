package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.teamcode.Command;

/**
 * Command that changes the robot's auto correction system's target angle.
 */
public class Turn extends Command {
    public Turn(double angle) {
        this.angle = Math.toRadians(angle);
    }
}
