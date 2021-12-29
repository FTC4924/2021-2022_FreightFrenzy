package org.firstinspires.ftc.teamcode.Commands;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.Constants.CommandType.BLUE_RED;

/**
 * Command that toggles between two sets of commands dependant on the alliance.
 */
public class BlueRed extends Command {
    public BlueRed(ArrayList<Command> blueCommands, ArrayList<Command> redCommands) {
        super(BLUE_RED);
        this.redCommands = redCommands;
        this.blueCommands = blueCommands;
    }
}
