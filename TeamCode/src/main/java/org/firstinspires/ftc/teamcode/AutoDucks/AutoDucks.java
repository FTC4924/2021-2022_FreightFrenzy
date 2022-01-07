package org.firstinspires.ftc.teamcode.AutoDucks;

import org.firstinspires.ftc.teamcode.AutoBase;
import org.firstinspires.ftc.teamcode.Commands.*;
import org.firstinspires.ftc.teamcode.Constants.AllianceColor;

import java.util.ArrayList;
import java.util.Arrays;

public abstract class AutoDucks extends AutoBase {
    protected abstract AllianceColor getAllianceColor();
    protected ArrayList<Command> getCommands() {
        return new ArrayList<Command>(
                Arrays.asList(
                        new Move(allianceColor.distanceToDucks, -90, .5),
                        new Pause(1),
                        new DetectDuckPosition(),
                        new Move(.5, -45.0, 0.5),
                        new Move(1.6, -86, .5),
                        new Ducks(),
                        new Pause(5),
                        new Ducks(),
                        new ArmRotate(.35),
                        new ArmExtend(.75),
                        new ArmFullRetract(),
                        new LoadDuckCommands(
                                new ArrayList<>(
                                        Arrays.asList(
                                                new ArmRotate(.5)
                                        )
                                ),
                                new ArrayList<>(
                                        Arrays.asList(
                                                new ArmRotate(.7)
                                        )
                                ),
                                new ArrayList<>(
                                        Arrays.asList(
                                                new ArmRotate(.95)
                                        )
                                )
                        ),
                        new Move(4.35,90,.5),
                        new Move(.8,0, .5),
                        new BristlesOut(),
                        new Pause(3),
                        new BristlesOut(),
                        new Turn(90),
                        new Move(2,90,1.5),
                        new ArmExtend(.2)
                        /*new Move(1, -45.0, 0.2),
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
                        new Pause(6),
                        new Ducks(),
                        new Move(1.8, -10, 1)
                         */
                )
        );
    }
}
