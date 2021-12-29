package org.firstinspires.ftc.teamcode.Commands;

import static org.firstinspires.ftc.teamcode.Constants.CommandType.DUCKS;

/**
 * Command for activating and deactivating the duck wheel.
 */
public class Ducks extends Command {
    public Ducks() {
        super(DUCKS);
        this.redCommands = redCommands;
        this.blueCommands = blueCommands;
    }
}
