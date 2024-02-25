package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class LEDSubsystem extends SubsystemBase {
  //private final AddressableLED m_led_left = new AddressableLED(RobotMap.LED_PWMPORT_LEFT);
  private final AddressableLED m_led_right = new AddressableLED(RobotMap.LED_PWMPORT_RIGHT);
  //private final AddressableLEDBuffer m_led_left_Buffer = new AddressableLEDBuffer(RobotMap.LED_LENGTH);
  private final AddressableLEDBuffer m_led_right_Buffer = new AddressableLEDBuffer(RobotMap.LED_LENGTH);
  //private int m_rainbowFirstPixelHue_left;
  private int m_rainbowFirstPixelHue_right;

  public LEDSubsystem() {
    //m_led_left.setLength(m_led_left_Buffer.getLength());
    ////m_led_left.setData(m_led_left_Buffer);
    //m_led_left.start();

    m_led_right.setLength(m_led_right_Buffer.getLength());
    m_led_right.setData(m_led_right_Buffer);
    m_led_right.start();
  }

  @Override
  public void periodic() {}

  public void rainbow() {
    // For every pixel
    /*
    for (var i = 0; i < m_led_left_Buffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue_left = (m_rainbowFirstPixelHue_left + (i * 180 / m_led_left_Buffer.getLength())) % 180;
      // Set the value
      m_led_left_Buffer.setHSV(i, hue_left, 255, 128);
    }
    */

     for (var i = 0; i < m_led_right_Buffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue_right = (m_rainbowFirstPixelHue_right + (i * 180 / m_led_right_Buffer.getLength())) % 180;
      // Set the value
      m_led_right_Buffer.setHSV(i, hue_right, 255, 128);
    }
    
    // Increase by to make the rainbow "move"
   // m_rainbowFirstPixelHue_left += 3;
    m_rainbowFirstPixelHue_right += 3;
    // Check bounds
    //m_rainbowFirstPixelHue_left %= 180;
    m_rainbowFirstPixelHue_right %= 180;
   // m_led_left.setData(m_led_left_Buffer);
  m_led_right.setData(m_led_right_Buffer);
  }

  private void setFrontAll(Color color) {
    /* 
    for (var i = 0; i < m_led_left_Buffer.getLength() / 2; i++) {
      m_led_left_Buffer.setLED(i, color);
    }
    */

    for (var i = 0; i < m_led_right_Buffer.getLength() / 2; i++) {
      m_led_right_Buffer.setLED(i, color);
    }
  }

 

  public void setBackAll(Color color) {
    /* 
    for (var i = m_led_left_Buffer.getLength() / 2; i < m_led_left_Buffer.getLength(); i++) {
      m_led_left_Buffer.setLED(i, color);
    }
    */

    for (var i = m_led_right_Buffer.getLength() / 2; i < m_led_right_Buffer.getLength(); i++) {
      m_led_right_Buffer.setLED(i, color);
    }
  }

  public void setAll(Color color) {
    /*
    for (var i = 0; i < m_led_left_Buffer.getLength(); i++) {
      m_led_left_Buffer.setLED(i, color);
    }
    */

    for (var i = 0; i < m_led_right_Buffer.getLength(); i++) {
      m_led_right_Buffer.setLED(i, color);
    }

   // m_led_left.setData(m_led_left_Buffer);
    m_led_right.setData(m_led_right_Buffer);
  }
}
