package org.firstinspires.ftc.teamcode.AutoDucks;

import org.firstinspires.ftc.teamcode.AutoBase;
import org.firstinspires.ftc.teamcode.Commands.*;
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
                        new Move(1, -45.0, 0.2),
                        new BlueRed(
                                new ArrayList<>(
                                        Arrays.asList(
                                                new Move(.3, 0, .5),
                                                new Turn(-90),
                                                new Move(4.4,90,.5)
                                        )
                                ),
                                new ArrayList<Command>(
                                        Arrays.asList(
                                                new Move(4.75, -86, .5)
                                        )
                                )
                        ),
                        new Ducks(),
                        new Wait(6),
                        new Ducks(),
                        new Move(1.8, -10, 1)
                )
        );
    }
}
