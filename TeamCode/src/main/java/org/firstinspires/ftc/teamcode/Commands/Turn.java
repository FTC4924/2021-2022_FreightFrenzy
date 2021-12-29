package org.firstinspires.ftc.teamcode.Commands;
import org.firstinspires.ftc.teamcode.Commands.Command;

import static org.firstinspires.ftc.teamcode.Constants.CommandType.TURN;

/**
 * Command that changes the robot's auto correction system's target angle.
 */
public class Turn extends Command {
    public Turn(double angle) {
        super(TURN);
        this.angle = Math.toRadians(angle);
    }
}
