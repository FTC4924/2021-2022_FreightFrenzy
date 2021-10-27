package org.firstinspires.ftc.teamcode;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.Constants.CommandType;

/**
 * Created by Brendan Clark on 09/24/2020 at 11:54 AM.
 */

public class Command {

    public CommandType commandType;
    public ArrayList<Command> noRingsCommands;
    public ArrayList<Command> oneRingCommands;
    public ArrayList<Command> fourRingsCommands;
    public double duration = 0;
    public double distance = 0;
    public double angle = 0;
    public double power = 0;

    public Command(CommandType commandType, AngleUnit angleUnit, double distance, double angle, double power) {
        this.commandType = commandType;
        this.distance = distance;
        if(angleUnit.bVal == 0) {
            this.angle = Math.toRadians(angle);
        } else {
            this.angle = angle;
        }
        this.power = power;
    }

    public Command(CommandType commandType, AngleUnit angleUnit, double angle) {
        this.commandType = commandType;
        if(angleUnit.bVal == 0) {
            this.angle = Math.toRadians(angle);
        } else {
            this.angle = angle;
        }
    }

    public Command(CommandType commandType, double duration) {
        this.commandType = commandType;
        this.duration = duration;
    }

    public Command(CommandType commandType) {
        this.commandType = commandType;
    }

    public Command(CommandType commandType, ArrayList<Command> noRingsCommands, ArrayList<Command> oneRingCommands, ArrayList<Command> fourRingsCommands) {
        this.commandType = commandType;
        this.noRingsCommands = noRingsCommands;
        this.oneRingCommands = oneRingCommands;
        this.fourRingsCommands = fourRingsCommands;
    }

}
