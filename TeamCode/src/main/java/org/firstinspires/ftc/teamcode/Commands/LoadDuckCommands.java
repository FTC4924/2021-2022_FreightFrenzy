package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.teamcode.Command;

import java.util.ArrayList;

public class LoadDuckCommands extends Command {
    public LoadDuckCommands(ArrayList<Command> leftCommands, ArrayList<Command> centerCommands, ArrayList<Command> rightCommands) {
        this.leftCommands = leftCommands;
        this.centerCommands = centerCommands;
        this.rightCommands = rightCommands;
    }
}
