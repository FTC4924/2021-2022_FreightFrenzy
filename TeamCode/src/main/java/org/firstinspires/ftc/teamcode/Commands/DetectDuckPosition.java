package org.firstinspires.ftc.teamcode.Commands;

import static org.firstinspires.ftc.teamcode.Constants.CommandType.DETECT_DUCK_POSITION;

/**
 * Command that uses the
 * <a href="#{@link}">{@link org.firstinspires.ftc.teamcode.visionpipelines.DuckDetectionPipeline}</a>
 * to detect the barcode position of the duck.
 * @see org.firstinspires.ftc.teamcode.visionpipelines.DuckDetectionPipeline
 */
public class DetectDuckPosition extends Command {
    public DetectDuckPosition() {
        super(DETECT_DUCK_POSITION);
    }
}
