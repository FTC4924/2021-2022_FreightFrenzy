package org.firstinspires.ftc.teamcode.Commands;


import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.Constants.CommandType;

/**
 * Created by Brendan Clark on 09/24/2020 at 11:54 AM.
 */

public class Command {

    public CommandType commandType;
    public ArrayList<Command> redCommands;
    public ArrayList<Command> blueCommands;
    public ArrayList<Command> leftCommands;
    public ArrayList<Command> centerCommands;
    public ArrayList<Command> rightCommands;
    public double duration = 0;
    public double distance = 0;
    public double angle = 0;
    public double power = 0;
    public int armRotation;
    public int armExtension;

    public Command(CommandType commandType, double distance, double angle, double power) {
        this.commandType = commandType;
        this.distance = distance;
        this.angle = Math.toRadians(angle);
        this.power = power;
    }

    public Command(CommandType commandType, double angle) {
        this.commandType = commandType;
        this.angle = Math.toRadians(angle);
        duration = angle;
    }

    public Command(CommandType commandType, int armPosition) {
        armRotation = armPosition;
        armExtension = armPosition;
    }

    public Command(CommandType commandType) {
        this.commandType = commandType;
    }

    public Command(CommandType commandType, ArrayList<Command> leftCommands, ArrayList<Command> centerCommands, ArrayList<Command> rightRingsCommands) {
        this.commandType = commandType;
        this.leftCommands = leftCommands;
        this.centerCommands = centerCommands;
        this.rightCommands = rightRingsCommands;
    }

    public Command(CommandType commandType, ArrayList<Command> blueCommands, ArrayList<Command> redCommands) {
        this.commandType = commandType;
        this.redCommands = redCommands;
        this.blueCommands = blueCommands;
    }

}
