package org.firstinspires.ftc.teamcode.Commands;

/**
 * Command that changes the robot's auto correction system's target angle.
 */
public class Turn extends Command {
    public Turn(double angle) {
        this.angle = Math.toRadians(angle);
    }
}
