package org.firstinspires.ftc.teamcode.Commands;

import java.util.ArrayList;

/**
 * Command that runs one set of commands based on the result of a previous call of
 * <a href="#{@link}">{@link DetectDuckPosition}</a>
 * @see DetectDuckPosition
 */
public class LoadDuckCommands extends Command {
    public LoadDuckCommands(ArrayList<Command> leftCommands, ArrayList<Command> centerCommands, ArrayList<Command> rightCommands) {
        this.leftCommands = leftCommands;
        this.centerCommands = centerCommands;
        this.rightCommands = rightCommands;
    }
}
