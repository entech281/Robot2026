package frc.entech.operatorpanel;

import edu.wpi.first.wpilibj.GenericHID;

public class OutputJoystick extends GenericHID {
    private final int[] ledValues = new int[4];
    private int lastSent = -1;

    public enum LedNumber { k0(0), k1(1), k2(2), k3(3); public final int v; LedNumber(int v){this.v=v;} }
    
    // Updated to support 3-bit colors (0-7)
    public enum Color { 
        BLACK(0), RED(1), GREEN(2), BLUE(3), 
        YELLOW(4), CYAN(5), MAGENTA(6), WHITE(7); 
        public final int v; Color(int v){this.v=v;} 
    }

    public OutputJoystick(int port) { super(port); }

    /**
     * @param color 3-bit color value
     * @param flash true = flashing, false = solid
     */
    public void setLED(LedNumber led, Color color, boolean flash) {
        // Bit 3: Flash, Bits 0-2: Color
        ledValues[led.v] = (flash ? 0x08 : 0x00) | (color.v & 0x07);
        flush();
    }

    public void flush() {
        int outputValue = 0;
        for (int i = 0; i < 4; i++) {
            // Pack 4 bits per LED into the 16-bit output
            outputValue |= (ledValues[i] & 0x0F) << (i * 4);
        }

        if (outputValue != lastSent) {
            // Mask to 16 bits to ensure clean HID output
            this.setOutputs(outputValue & 0xFFFF);
            lastSent = outputValue;
        }
    }

    public void clearAll() {
        for (int i = 0; i < 4; i++) ledValues[i] = 0;
        flush();
    }
}