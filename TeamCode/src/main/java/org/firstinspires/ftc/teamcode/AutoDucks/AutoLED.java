package org.firstinspires.ftc.teamcode.AutoDucks;

import org.firstinspires.ftc.teamcode.AutoBase;
import org.firstinspires.ftc.teamcode.Commands.*;
import org.firstinspires.ftc.teamcode.Constants.AllianceColor;

import java.util.ArrayList;
import java.util.Arrays;

public abstract class AutoLED extends AutoBase {
    protected abstract AllianceColor getAllianceColor();
    protected ArrayList<Command> getCommands() {
        return new ArrayList<Command>(
                Arrays.asList(
                        new Pause(100)
                )
        );
    }
}
