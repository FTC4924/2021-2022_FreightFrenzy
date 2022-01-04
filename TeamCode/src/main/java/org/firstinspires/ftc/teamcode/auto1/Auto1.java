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
                        new ArmRotate(.35),
                        new ArmExtend(.5),
                        new Move(allianceColor.distanceToDucks, -90, .5),
                        new DetectDuckPosition(),
                        new Move(0.25,0.5),
                        new Turn(32),
                        new Move(1,.5)/*,
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
                                )
                        )*/
                )
        );
    }
}
