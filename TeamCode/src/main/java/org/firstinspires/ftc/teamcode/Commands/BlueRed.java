package org.firstinspires.ftc.teamcode.Commands;

import org.firstinspires.ftc.teamcode.Command;

import java.util.ArrayList;

public class BlueRed extends Command {
    public BlueRed(ArrayList<Command> blueCommands, ArrayList<Command> redCommands) {
        this.blueCommands = blueCommands;
        this.redCommands = redCommands;
    }
}
