package LoadSensorI2cDriver;

import com.qualcomm.robotcore.hardware.I2cAddr;

public class Constants {

    protected static final I2cAddr ADDRESS_I2C_DEFAULT = I2cAddr.create7bit(0x2A);

    protected enum Register {
        READ((byte)0x12),
        CTRL2((byte)0x02),
        PU_CTRL((byte)0x00);

        public byte bVal;

        Register(byte value) { this.bVal = value; }
    }


    protected enum PU_CTRL_Bits{
        PU_CTRL_RR,
        PU_CTRL_PUD,
        PU_CTRL_PUA,
        PU_CTRL_PUR,
        PU_CTRL_CS,
        PU_CTRL_CR,
        PU_CTRL_OSCS,
        PU_CTRL_AVDDS
    }

    protected static final int SAMPLE_SIZE = 8;
    protected static final int DEFAULT_ZERO_OFFSET = 0;
    protected static final double DEFAULT_CALIBRATION_FACTOR = 0;
    protected static final byte DEFAULT_SAMPLE_RATE = 0b111;
}
