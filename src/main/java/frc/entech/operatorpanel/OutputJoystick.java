package frc.entech.operatorpanel;

import edu.wpi.first.wpilibj.GenericHID;

public class OutputJoystick extends GenericHID {
  // Internal state to track what's currently set for each LED
  private final int[] ledValues = new int[4]; // Stores the 4-bit nibble for each LED

  public enum LedNumber { k0(0), k1(1), k2(2), k3(3); public final int v; LedNumber(int v){this.v=v;} }
  public enum Color { BLACK(0), RED(1), GREEN(2), BLUE(3); public final int v; Color(int v){this.v=v;} }
  public enum BlinkRate { SOLID(0), SLOW(1), MEDIUM(2), FAST(3); public final int v; BlinkRate(int v){this.v=v;} }

  public OutputJoystick(int port) {
    super(port);
  }

  public void setLED(LedNumber led, Color color, BlinkRate blink) {
    // 1. Pack color and blink into a 4-bit nibble (Color=bits 0-1, Blink=bits 2-3)
    int nibble = (blink.v << 2) | (color.v & 0x03);
    ledValues[led.v] = nibble;

    // 2. Pack LEDs into Bytes
    // Byte 0 (Left Rumble): LED 1 (High) | LED 0 (Low)
    int byte0 = (ledValues[1] << 4) | (ledValues[0] & 0x0F);
    // Byte 1 (Right Rumble): LED 3 (High) | LED 2 (Low)
    int byte1 = (ledValues[3] << 4) | (ledValues[2] & 0x0F);

    // 3. Convert bytes (0-255) to WPILib doubles (0.0-1.0)
    // We divide by 255.0 so 255 becomes 1.0 and 128 becomes ~0.5
    this.setRumble(RumbleType.kLeftRumble, byte0 / 255.0);
    this.setRumble(RumbleType.kRightRumble, byte1 / 255.0);
  }
}
