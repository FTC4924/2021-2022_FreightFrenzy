package LoadSensorI2cDriver;

import com.qualcomm.robotcore.hardware.I2cAddr;

public class Constants {

    protected static final I2cAddr ADDRESS_I2C_DEFAULT = I2cAddr.create7bit(0x2A);

    protected enum Register {
        PU_CTRL((byte)0x00),
        CTRL1((byte)0x01),
        CTRL2((byte)0x02),
        ADCO_B2((byte)0x12),
        ADC((byte)0x15),
        PGA_PWR((byte)0x1C);

        public byte bVal;
        Register(byte value) { this.bVal = value; }
    }


    protected enum PU_CTRL_Bits{
        PU_CTRL_RR((byte) 0),
        PU_CTRL_PUD((byte) 1),
        PU_CTRL_PUA((byte) 2),
        PU_CTRL_PUR((byte) 3),
        PU_CTRL_CS((byte) 4),
        PU_CTRL_CR((byte) 5),
        PU_CTRL_OSCS((byte) 6),
        PU_CTRL_AVDDS((byte) 7);

        public byte index;
        PU_CTRL_Bits(byte index) { this.index = index; }
    }

    protected enum CTRL2_BITS {
        CTRL2_CALMOD((byte) 0),
        CTRL2_CALS((byte) 2),
        CTRL2_CAL_ERROR((byte) 3),
        CTRL2_CRS((byte) 4),
        CTRL2_CHS((byte) 7);

        public byte index;
        CTRL2_BITS(byte index) { this.index = index; }
    }

    protected enum PGA_PWR_Bits {
        PGA_PWR_PGA_CURR((byte)0),
        PGA_PWR_ADC_CURR((byte)2),
        PGA_PWR_MSTR_BIAS_CURR((byte)4),
        PGA_PWR_PGA_CAP_EN((byte)7);

        public byte index;
        PGA_PWR_Bits(byte index) { this.index = index; }
    }

    protected enum LDO_VALUES {
        LDO_2V4((byte) 0b111),
        LDO_2V7((byte) 0b110),
        LDO_3V0((byte) 0b101),
        LDO_3V3((byte) 0b100),
        LDO_3V6((byte) 0b011),
        LDO_3V9((byte) 0b010),
        LDO_4V2((byte) 0b001),
        LDO_4V5((byte) 0b000);

        byte value;

        LDO_VALUES(byte value) { this.value = value; }
    }

    protected enum GAIN_VALUES {
        GAIN_128((byte) 0b111),
        GAIN_64((byte) 0b110),
        GAIN_32((byte) 0b101),
        GAIN_16((byte) 0b100),
        GAIN_8((byte) 0b011),
        GAIN_4((byte) 0b010),
        GAIN_2((byte) 0b001),
        GAIN_1((byte) 0b000);

        byte value;
        GAIN_VALUES(byte value) { this.value = value; }
    }

    protected enum SPS_VALUES {
        SPS_320((byte) 0b111),
        SPS_80((byte) 0b011),
        SPS_40((byte) 0b010),
        SPS_20((byte) 0b001),
        SPS_10((byte) 0b000);

        byte value;
        SPS_VALUES(byte value) { this.value = value; }
    }

    protected static final int SAMPLE_SIZE = 10;
    protected static final int DEFAULT_ZERO_OFFSET = 0;
    protected static final double DEFAULT_CALIBRATION_FACTOR = 1;
    protected static final byte DEFAULT_SAMPLE_RATE = 0b111;
}
