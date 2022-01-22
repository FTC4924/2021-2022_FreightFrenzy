package org.firstinspires.ftc.teamcode.Commands;

import java.util.ArrayList;

/**
 * Command that runs one set of commands based on the result of a previous call of
 * <a href="#{@link}">{@link DetectBarcodePosition}</a>
 * @see DetectBarcodePosition
 */
public class LoadBarcodeCommands extends Command {
    public LoadBarcodeCommands(ArrayList<Command> leftCommands, ArrayList<Command> centerCommands, ArrayList<Command> rightCommands) {
        this.leftCommands = leftCommands;
        this.centerCommands = centerCommands;
        this.rightCommands = rightCommands;
    }
}
