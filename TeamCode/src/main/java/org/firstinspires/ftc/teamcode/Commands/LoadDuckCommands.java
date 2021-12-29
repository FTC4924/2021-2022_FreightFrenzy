package org.firstinspires.ftc.teamcode.Commands;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.Constants.CommandType.LOAD_DUCK_COMMANDS;

/**
 * Command that runs one set of commands based on the result of a previous call of
 * <a href="#{@link}">{@link DetectDuckPosition}</a>
 * @see DetectDuckPosition
 */
public class LoadDuckCommands extends Command {
    public LoadDuckCommands(ArrayList<Command> leftCommands, ArrayList<Command> centerCommands, ArrayList<Command> rightRingsCommands) {
        super(LOAD_DUCK_COMMANDS);
        this.leftCommands = leftCommands;
        this.centerCommands = centerCommands;
        this.rightCommands = rightRingsCommands;
    }
}