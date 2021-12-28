package org.firstinspires.ftc.teamcode.auto1;

import org.firstinspires.ftc.teamcode.AutoBase;
import org.firstinspires.ftc.teamcode.Constants.AllianceColor;
import org.firstinspires.ftc.teamcode.Commands.*;

import java.util.ArrayList;
import java.util.Arrays;

import static org.firstinspires.ftc.teamcode.Constants.CommandType.*;


public abstract class Auto1 extends AutoBase {
    protected abstract AllianceColor getAllianceColor();
    protected ArrayList<Command> getCommands() {
        AllianceColor allianceColor = getAllianceColor();
        return new ArrayList<>(
                Arrays.asList(
                        new Move(allianceColor.distanceToDucks, -90, .5),
                        new Command(DETECT_DUCK_POSITION),
                        new Command(TURN,25),
                        new Command(MOVE,3,0,.5),
                        new Command(LOAD_DUCK_COMMANDS,
                                new ArrayList<>(
                                        Arrays.asList(
                                                new Command(MOVE,0.45, 90, .5)
                                        )
                                ),
                                new ArrayList<>(
                                        Arrays.asList(
                                                new Command(MOVE,0.5, -90, .5)
                                        )
                                ),
                                new ArrayList<>(
                                        Arrays.asList(
                                                new Command(MOVE,0.5, -90, .5)
                                        )
                                ))
                )
        );
    }
}
