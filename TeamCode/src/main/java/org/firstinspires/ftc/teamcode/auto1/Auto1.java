package org.firstinspires.ftc.teamcode.auto1;

import org.firstinspires.ftc.teamcode.AutoBase;
import org.firstinspires.ftc.teamcode.Command;
import org.firstinspires.ftc.teamcode.Constants.AllianceColor;
import org.firstinspires.ftc.teamcode.Commands.*;

import java.util.ArrayList;
import java.util.Arrays;

public abstract class Auto1 extends AutoBase {
    protected abstract AllianceColor getAllianceColor();
    protected ArrayList<Command> getCommands() {
        return new ArrayList<>(
                Arrays.asList(
                        new Move(allianceColor.distanceToDucks, -90, .5),
                        new DetectDuckPosition(),
                        new Turn(25),
                        new Move(3,0,.5),
                        new LoadDuckCommands(
                                new ArrayList<>(
                                        Arrays.asList(
                                                new Move(0.45, 90, .5)
                                        )
                                ),
                                new ArrayList<>(
                                        Arrays.asList(
                                                new Move(0.5, -90, .5)
                                        )
                                ),
                                new ArrayList<>(
                                        Arrays.asList(
                                                new Move(0.5, -90, .5)
                                        )
                                ))
                )
        );
    }
}
