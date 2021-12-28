package org.firstinspires.ftc.teamcode.AutoDucks;

import org.firstinspires.ftc.teamcode.AutoBase;
import org.firstinspires.ftc.teamcode.Commands.Command;
import org.firstinspires.ftc.teamcode.Constants.AllianceColor;

import java.util.ArrayList;
import java.util.Arrays;

import static org.firstinspires.ftc.teamcode.Constants.CommandType.*;


public abstract class AutoDucks extends AutoBase {
    protected abstract AllianceColor getAllianceColor();
    protected ArrayList<Command> getCommands() {
        AllianceColor allianceColor = getAllianceColor();
        return new ArrayList<>(
                Arrays.asList(
                        new Command(MOVE, 1, -45.0, 0.2),
                        new Command(BLUE_RED,
                                new ArrayList<>(
                                        Arrays.asList(
                                                new Command(MOVE, .3, 0, .5),
                                                new Command(TURN, -90),
                                                new Command(MOVE, 4.4,90,.5)
                                        )
                                ),
                                new ArrayList<>(
                                        Arrays.asList(
                                                new Command(MOVE, 4.75, -86, .5)
                                        )
                                )
                        ),
                        new Command(DUCKS),
                        new Command(WAIT, 6),
                        new Command(DUCKS),
                        new Command(MOVE, 1.8, -10, 1)
                )
        );
    }
}
