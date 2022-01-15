package org.firstinspires.ftc.teamcode.auto2;

import org.firstinspires.ftc.teamcode.AutoBase;
import org.firstinspires.ftc.teamcode.Commands.ArmExtend;
import org.firstinspires.ftc.teamcode.Commands.ArmFullRetract;
import org.firstinspires.ftc.teamcode.Commands.ArmRotate;
import org.firstinspires.ftc.teamcode.Commands.BristlesOut;
import org.firstinspires.ftc.teamcode.Commands.Command;
import org.firstinspires.ftc.teamcode.Commands.DetectDuckPosition;
import org.firstinspires.ftc.teamcode.Commands.LoadDuckCommands;
import org.firstinspires.ftc.teamcode.Commands.Move;
import org.firstinspires.ftc.teamcode.Commands.Pause;
import org.firstinspires.ftc.teamcode.Commands.Turn;
import org.firstinspires.ftc.teamcode.Constants;

import java.util.ArrayList;
import java.util.Arrays;

public abstract class Auto2 extends AutoBase {
    protected abstract Constants.AllianceColor getAllianceColor();
    protected ArrayList<Command> getCommands() {
        return new ArrayList<>(
                Arrays.asList(
                        new Move(allianceColor.distanceToDucks, 90, .5),
                        new Pause(1),
                        new DetectDuckPosition(),
                        new ArmRotate(.35),
                        new ArmExtend(.75),
                        new ArmFullRetract(),
                        new Move(0.25, 0, 0.5),
                        new Turn(-34),
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
                                                new ArmRotate(.95)
                                        )
                                )
                        ),
                        new Move(1.75,-34,.5),
                        new BristlesOut(),
                        new Pause(5),
                        new BristlesOut()
                )
        );
    }
}

