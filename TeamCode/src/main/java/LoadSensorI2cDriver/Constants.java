package LoadSensorI2cDriver;

import com.qualcomm.robotcore.hardware.I2cAddr;

public class Constants {

    protected static final I2cAddr ADDRESS_I2C_DEFAULT = I2cAddr.create7bit(0x70);

    protected enum Command {
        READ((byte)0x12);
        public byte bVal;
        Command(byte value) { this.bVal = value; }
    }
}
