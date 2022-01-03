package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;

/**
 * Created by Brendan Clark on 09/24/2020 at 11:54 AM.
 */

public class Command {

    public ArrayList<Command> redCommands;
    public ArrayList<Command> blueCommands;
    public ArrayList<Command> leftCommands;
    public ArrayList<Command> centerCommands;
    public ArrayList<Command> rightCommands;
    public double duration = 0;
    public double distance = 0;
    public double angle = 0;
    public double power = 0;
    public int position;

}
