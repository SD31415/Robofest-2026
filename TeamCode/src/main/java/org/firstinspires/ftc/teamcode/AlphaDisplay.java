package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;

@SuppressWarnings({"weakerAccess", "unused"})
@I2cDeviceType
@DeviceProperties(name = "Dual Alphanumeric Display", xmlTag = "AlphaDisplay")
public class AlphaDisplay extends I2cDeviceSynchDevice<I2cDeviceSynch>{
    public final static I2cAddr ADDRESS_I2C_DEFAULT = I2cAddr.create7bit(0x70);
    private static final int SYSTEM_ON = 0x21;
    private static final int DISPLAY_DATA = 0x00;
    private static final int DISPLAY_ON = 0x81;
    private final byte[] display = new byte[] {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    private static final int[] alphaFontTable = new int[] {
        0b0000000000000001, 0b0000000000000010, 0b0000000000000100,
        0b0000000000001000, 0b0000000000010000, 0b0000000000100000,
        0b0000000001000000, 0b0000000010000000, 0b0000000100000000,
        0b0000001000000000, 0b0000010000000000, 0b0000100000000000,
        0b0001000000000000, 0b0010000000000000, 0b0100000000000000,
        0b1000000000000000, 0b0000000000000000, 0b0000000000000000,
        0b0000000000000000, 0b0000000000000000, 0b0000000000000000,
        0b0000000000000000, 0b0000000000000000, 0b0000000000000000,
        0b0001001011001001, 0b0001010111000000, 0b0001001011111001,
        0b0000000011100011, 0b0000010100110000, 0b0001001011001000,
        0b0011101000000000, 0b0001011100000000,
        0b0000000000000000, //
        0b0000000000000110, // !
        0b0000001000100000, // "
        0b0001001011001110, // #
        0b0001001011101101, // $
        0b0000110000100100, // %
        0b0010001101011101, // &
        0b0000010000000000, // '
        0b0010010000000000, // (
        0b0000100100000000, // )
        0b0011111111000000, // *
        0b0001001011000000, // +
        0b0000100000000000, // ,
        0b0000000011000000, // -
        0b0100000000000000, // .
        0b0000110000000000, // /
        0b0000110000111111, // 0
        0b0000000000000110, // 1
        0b0000000011011011, // 2
        0b0000000010001111, // 3
        0b0000000011100110, // 4
        0b0010000001101001, // 5
        0b0000000011111101, // 6
        0b0000000000000111, // 7
        0b0000000011111111, // 8
        0b0000000011101111, // 9
        0b0001001000000000, // :
        0b0000101000000000, // ;
        0b0010010000000000, // <
        0b0000000011001000, // =
        0b0000100100000000, // >
        0b0001000010000011, // ?
        0b0000001010111011, // @
        0b0000000011110111, // A
        0b0001001010001111, // B
        0b0000000000111001, // C
        0b0001001000001111, // D
        0b0000000011111001, // E
        0b0000000001110001, // F
        0b0000000010111101, // G
        0b0000000011110110, // H
        0b0001001000001001, // I
        0b0000000000011110, // J
        0b0010010001110000, // K
        0b0000000000111000, // L
        0b0000010100110110, // M
        0b0010000100110110, // N
        0b0000000000111111, // O
        0b0000000011110011, // P
        0b0010000000111111, // Q
        0b0010000011110011, // R
        0b0000000011101101, // S
        0b0001001000000001, // T
        0b0000000000111110, // U
        0b0000110000110000, // V
        0b0010100000110110, // W
        0b0010110100000000, // X
        0b0001010100000000, // Y
        0b0000110000001001, // Z
        0b0000000000111001, // [
        0b0010000100000000, //
        0b0000000000001111, // ]
        0b0000110000000011, // ^
        0b0000000000001000, // _
        0b0000000100000000, // `
        0b0001000001011000, // a
        0b0010000001111000, // b
        0b0000000011011000, // c
        0b0000100010001110, // d
        0b0000100001011000, // e
        0b0000000001110001, // f
        0b0000010010001110, // g
        0b0001000001110000, // h
        0b0001000000000000, // i
        0b0000000000001110, // j
        0b0011011000000000, // k
        0b0000000000110000, // l
        0b0001000011010100, // m
        0b0001000001010000, // n
        0b0000000011011100, // o
        0b0000000101110000, // p
        0b0000010010000110, // q
        0b0000000001010000, // r
        0b0010000010001000, // s
        0b0000000001111000, // t
        0b0000000000011100, // u
        0b0010000000000100, // v
        0b0010100000010100, // w
        0b0010100011000000, // x
        0b0010000000001100, // y
        0b0000100001001000, // z
        0b0000100101001001, // {
        0b0001001000000000, // |
        0b0010010010001001, // }
        0b0000010100100000, // ~
        0b0011111111111111,
    };
    public AlphaDisplay(I2cDeviceSynch i2cDeviceSynch, boolean deviceClientIsOwned) {
        super(i2cDeviceSynch, deviceClientIsOwned);
        this.deviceClient.setI2cAddress(ADDRESS_I2C_DEFAULT);
        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }

    @Override
    protected boolean doInitialize() {
        deviceClient.write8(SYSTEM_ON);
        updateDisplay();
        deviceClient.write8(DISPLAY_ON);
        return true;
    }

    public void updateDisplay() {
        deviceClient.write(DISPLAY_DATA, display);
    }
    public void writeCharacter(char character, int position, boolean dot) {
        int code = alphaFontTable[character];
        if (dot) {
            code |= (1 << 14);
        }
        display[position*2] = (byte)(code & 0xFF);
        display[position*2+1] = (byte)(code >> 8);
    }

    public void writeNumber(int number) {
        int ones = number % 10;
        writeCharacter((char) ('0' + ones), 3, false);
        int tens = (number / 10) % 10;
        writeCharacter((char) ('0' + tens), 2, false);
        int hundreds = (number / 100) % 10;
        writeCharacter((char) ('0' + hundreds), 1, false);
        int thousands = (number / 1000) % 10;
        writeCharacter((char) ('0' + thousands), 0, false);
        updateDisplay();
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Adafruit;
    }

    @Override
    public String getDeviceName() {
        return "Dual Alphanumeric Display";
    }
}