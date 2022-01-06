package org.firstinspires.ftc.teamcode.auto1;

import org.firstinspires.ftc.teamcode.AutoBase;
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
                        new Pause(1),
                        new DetectDuckPosition(),
                        new ArmRotate(.35),
                        new ArmExtend(.75),
                        new ArmFullRetract(),
                        new Move(0.25, 0, 0.5),
                        new Turn(33),
                        new LoadDuckCommands(
                                new ArrayList<>(
                                        Arrays.asList(
                                                new ArmRotate(.5)
                                        )
                                ),
                                new ArrayList<>(
                                        Arrays.asList(
                                                new ArmRotate(.75)
                                        )
                                ),
                                new ArrayList<>(
                                        Arrays.asList(
                                                new ArmRotate(1)
                                        )
                                )
                        ),
                        new Move(1.75,33,.5),
                        new BristlesOut(),
                        new Pause(5),
                        new BristlesOut()
                )
        );
    }
}
