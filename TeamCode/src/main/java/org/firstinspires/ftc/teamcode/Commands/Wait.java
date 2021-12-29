package org.firstinspires.ftc.teamcode.Commands;

import static org.firstinspires.ftc.teamcode.Constants.CommandType.WAIT;

/**
 * Command that holds the state machine for a duration.
 */
public class Wait extends Command {
    public Wait(double duration) {
        super(WAIT);
        this.duration = duration;
    }
}
