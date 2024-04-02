package frc.robot.commands;

import frc.robot.subsystems.LEDSubsystem;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

// TODO remove println's, format code, remove dead code, rewiew intake/shoot patterns, clean up warning
public class LEDCommand extends Command {

      private static final int LIGHT_DELAY_NUM_OF_ITERATIONS = 32;

      // ----- CONSTANT(S) -----\\
      private int counter = 0;
      private boolean animCheck = false;
      private int blueOffIndex = 0;
      private int blueOnIndex = 19;
      private int yellowOffIndex = 30;
      private int yellowOnIndex = 49;
      private int blueIndex = 0;
      private int greenIndex = 19;
      private Alliance allianceColor;
      // The LED Subsystem (strip) itself
      private final LEDSubsystem m_LEDSubsystem;
      private final LEDPatterns m_pattern;
      // ------CONSTUCTOR(S)--------\\
      private final AddressableLEDBuffer m_buffer;

      public LEDCommand(LEDSubsystem subsystem, LEDPatterns pattern) {
            m_pattern = pattern;
            m_LEDSubsystem = subsystem;
            m_buffer = m_LEDSubsystem.getBuffer();
            //allianceColor = DriverStation.getAlliance();
            addRequirements(m_LEDSubsystem);
      }

      @Override
      public void initialize() {
            // force intialize in state to publish lights (at delay counter)
            counter = LIGHT_DELAY_NUM_OF_ITERATIONS;
            // Alliance can change during simulation
            //allianceColor = DriverStation.getAlliance();
            // TODO convert to switch
            if (m_pattern == LEDPatterns.Easy) {
                  easyInit();
            } else if (m_pattern == LEDPatterns.EveryOther) {
                  everyOtherInit();
            } else if (m_pattern == LEDPatterns.IdlePattern) {
                  idlePatternInit();
            } else if (m_pattern == LEDPatterns.IntakePattern) {
                  intakePatternInit();
            } else if (m_pattern == LEDPatterns.ShooterPattern) {
                  shooterPatternInit();
            } else if (m_pattern == LEDPatterns.OnEveryOther) {
                  onEveryOtherInit();
            }
      }

      @Override
      public void execute() {
            // TODO convert to switch
            if (m_pattern == LEDPatterns.Easy) {
                  easy();
            } else if (m_pattern == LEDPatterns.EveryOther) {
                  everyOther();
            } else if (m_pattern == LEDPatterns.IdlePattern) {
                  idlePattern();
            } else if (m_pattern == LEDPatterns.IntakePattern) {
                  intakePattern();
            } else if (m_pattern == LEDPatterns.ShooterPattern) {
                  shooterPattern();
            } else if (m_pattern == LEDPatterns.OnEveryOther) {
                  OnEveryOther();
            } else if (m_pattern == LEDPatterns.SolidLEDs) {
                  SolidLEDs();
            }
      }

      // Called once the command ends or is interrupted.
      @Override
      public void end(boolean interrupted) {
            ledsOff();
      } // End of end()

      // Returns true when the command should end.
      @Override
      public boolean isFinished() {
            return false;
      } // End of isFinished()

      // Enum for calculated target size
      public static enum LEDPatterns {
            Easy, EveryOther, IdlePattern, IntakePattern, ShooterPattern, OnEveryOther, SolidLEDs;
            // private int type;

            private LEDPatterns() {
            }
      }

      private void moveLEDs() {
            // Increments all index positions
            blueOffIndex++;
            blueOnIndex++;
            yellowOffIndex++;
            yellowOnIndex++;

            // If said index variable is greater than or equal to
            // m_LEDSubsystem.getBufferLength(), set it to 0
            if (blueOnIndex >= m_LEDSubsystem.getBufferLength()) {
                  blueOnIndex = 0;
            }
            if (blueOffIndex >= m_LEDSubsystem.getBufferLength()) {
                  blueOffIndex = 0;
            }
            if (yellowOffIndex >= m_LEDSubsystem.getBufferLength()) {
                  yellowOffIndex = 0;
            }
            if (yellowOnIndex >= m_LEDSubsystem.getBufferLength()) {
                  yellowOnIndex = 0;
            }
      } // End of moveLEDs()

      private void moveBGLEDs() {
            // Increments all index positions
            blueIndex++;
            greenIndex++;

            // If said index variable is greater than or equal to
            // m_LEDSubsystem.getBufferLength(), set it to 0
            if (blueIndex >= m_LEDSubsystem.getBufferLength()) {
                  blueIndex = 0;
            }
            if (greenIndex >= m_LEDSubsystem.getBufferLength()) {
                  greenIndex = 0;
            }
      } // End of ()

      public void easyInit() {
            // RGB values for blue and red
            int r[] = { 0, 150 };
            int g[] = { 0, 0 };
            int b[] = { 150, 0 };
            int index = (allianceColor == Alliance.Blue) ? 0 : 1;
            for (int i = 0; i < m_LEDSubsystem.getBufferLength(); i += 2) {
                  m_buffer.setRGB(i, r[index], g[index], b[index]);
            }
            m_LEDSubsystem.setBuffer(m_buffer);

      }

      public void easy() {
      }

      public void everyOtherInit() {
      }

      public void onEveryOtherInit() {

      }

      public void everyOther() {
            counter++;
            if (counter >= LIGHT_DELAY_NUM_OF_ITERATIONS) {

                  // BLUE ALLIANCE
                  if (allianceColor == Alliance.Blue) {

                        if (animCheck == true) {

                              /**
                               * sets the LED pattern to:
                               * 10101010101010......
                               * 
                               * (0 being off, 1 being blue)
                               */
                              for (int i = 0; i < m_LEDSubsystem.getBufferLength(); i++) {
                                    if (i % 2 == 0) { // Sets every other LED to bright blue
                                          m_buffer.setRGB(i, 0, 0, 255);
                                    } else {
                                          m_buffer.setRGB(i, 0, 0, 0);
                                    }
                              }

                              // Sets animCheck to false for pattern switch next time the delay is finished
                              animCheck = false;

                        } else {

                              /**
                               * Clears the LED strip and sets all LED's to off
                               */
                              for (int i = 0; i < m_LEDSubsystem.getBufferLength(); i++) {
                                    m_buffer.setRGB(i, 0, 0, 0);
                              }

                              /**
                               * Sets the LED pattern to:
                               * 01010101010101......
                               * 
                               * (0 being off, 1 being blue)
                               */
                              for (int i = 1; i < m_LEDSubsystem.getBufferLength(); i += 2) { // Sets every other LED to
                                                                                              // bright blue
                                    m_buffer.setRGB(i, 0, 0, 255);
                              }

                              // Sets animCheck to true for pattern switch next time the delay is finished
                              animCheck = true;

                        }

                  }

                  // RED ALLIANCE
                  if (allianceColor == Alliance.Red) {

                        if (animCheck == true) {

                              /**
                               * sets the LED pattern to:
                               * 10101010101010......
                               * 
                               * (0 being off, 1 being red)
                               */
                              for (int i = 0; i < m_LEDSubsystem.getBufferLength(); i++) {
                                    if (i % 2 == 0) { // Sets every other LED to bright red
                                          m_buffer.setRGB(i, 255, 0, 0);
                                    } else {
                                          m_buffer.setRGB(i, 0, 0, 0);
                                    }
                              }

                              // Sets animCheck to false for pattern switch next time the delay is finished
                              animCheck = false;

                        } else {

                              /**
                               * clears the LED strip and sets all LED's to off
                               */
                              for (int i = 0; i < m_LEDSubsystem.getBufferLength(); i++) {
                                    m_buffer.setRGB(i, 0, 0, 0);
                              }

                              /**
                               * Sets the LED pattern to:
                               * 01010101010101......
                               * 
                               * (0 being off, 1 being red)
                               */
                              for (int i = 1; i < m_LEDSubsystem.getBufferLength(); i += 2) { // Sets every other LED to
                                                                                              // bright red
                                    m_buffer.setRGB(i, 255, 0, 0);
                              }

                              // Sets animCheck to true for pattern switch next time the delay is finished
                              animCheck = true;

                        }

                  }
                  m_LEDSubsystem.setBuffer(m_buffer);
                  // Resets the counter to 0 once the pattern is set
                  counter = 0;
            }
      }

      public void idlePatternInit() {
            System.out.println("IdlePatternInit:" + counter);
      }

      public void idlePattern() {
            System.out.println("IdlePattern:" + counter);

            counter++;

            // If counter is equal or greater to LIGHT_DELAY_NUM_OF_ITERATIONS (basically
            // acting as a delay)
            if (counter >= LIGHT_DELAY_NUM_OF_ITERATIONS) {
                  animCheck = !animCheck;
                  counter = 0;
            }
            if (animCheck == true) {
                  /**
                   * sets the LED pattern to:
                   * 10101010101010......
                   * 
                   * (0 being off, 1 being yellow)
                   */
                  for (int i = 0; i < m_LEDSubsystem.getBufferLength(); i += 2) {
                        m_buffer.setRGB(i, 150, 150, 0); // Yellow
                        m_buffer.setRGB(i + 1, 0, 0, 150); // Blue
                  }
            } else {
                  /**
                   * sets the LED pattern to:
                   * 01010101010101......
                   * 
                   * (0 being off, 1 being yellow)
                   */
                  for (int i = 1; i < m_LEDSubsystem.getBufferLength(); i += 2) {
                        m_buffer.setRGB(i, 150, 150, 0); // Yellow
                        m_buffer.setRGB(i - 1, 0, 0, 150); // Blue
                  }
            }
            m_LEDSubsystem.setBuffer(m_buffer);
      }

      public void intakePatternInit() {
            System.out.println("IntakeInit:" + counter);
      }

      public void intakePattern() {
            System.out.println("Intake:" + counter);
            counter++;

            // If counter is equal or greater to 2 (basically acting as a delay)
            if (counter >= LIGHT_DELAY_NUM_OF_ITERATIONS) {
                  for (int ii = 0; ii < m_LEDSubsystem.getBufferLength() + 3; ii++) {

                        m_buffer.setRGB(blueOffIndex, 0, 0, 0);
                        m_buffer.setRGB(yellowOffIndex, 0, 0, 0);
                        moveLEDs(); // Moves the LEDs
                        m_buffer.setRGB(blueOnIndex, 0, 0, 255); // Blue
                        m_buffer.setRGB(yellowOnIndex, 255, 255, 0); // Yellow
                  }

                  // Resets the counter to 0 once the pattern is set
                  counter = 0;
                  m_LEDSubsystem.setBuffer(m_buffer);
            }
      }

      public void shooterPatternInit() {
            System.out.println("ShootPatternInit:" + counter);
            blueIndex = 0;
            greenIndex = 19;
      }

      public void shooterPattern() {
            System.out.println("ShootPattern:" + counter);
            counter++;
            // Keeping track of animation speed.
            if (counter >= LIGHT_DELAY_NUM_OF_ITERATIONS) {
                  for (int ii = 0; ii < m_LEDSubsystem.getBufferLength() + 3; ii++) {
                        m_buffer.setRGB(blueIndex, 0, 0, 255); // Sets LED at blueIndex to blue
                        moveBGLEDs(); // Moves the LEDs
                        m_buffer.setRGB(greenIndex, 0, 255, 0); // Sets LED at greenIndex to green
                  }

                  counter = 0;
                  m_LEDSubsystem.setBuffer(m_buffer);
            }

      }

      public void ledsOff() {

            for (int i = 0; i < m_LEDSubsystem.getBufferLength(); i++) {
                  m_buffer.setRGB(i, 0, 0, 0); // off
            }
            m_LEDSubsystem.setBuffer(m_buffer);
      }

      public void OnEveryOther() {
            counter++;
            if (counter >= LIGHT_DELAY_NUM_OF_ITERATIONS) {

                  // BLUE ALLIANCE
                  if (allianceColor == Alliance.Blue) {

                        if (animCheck == true) {

                              /**
                               * sets the LED pattern to:
                               * 10101010101010......
                               * 
                               * (0 being off, 1 being blue)
                               */
                              for (int i = 0; i < m_LEDSubsystem.getBufferLength(); i++) {
                                    if (i % 2 == 0) { // Sets every other LED to bright blue
                                          m_buffer.setRGB(i, 0, 0, 255);
                                    } else {
                                          m_buffer.setRGB(i, 150, 150, 150);
                                    }
                              }

                              // Sets animCheck to false for pattern switch next time the delay is finished
                              animCheck = false;

                        } else {

                              /**
                               * Clears the LED strip and sets all LED's to off
                               */
                              for (int i = 0; i < m_LEDSubsystem.getBufferLength(); i++) {
                                    m_buffer.setRGB(i, 150, 150, 150);
                              }

                              /**
                               * Sets the LED pattern to:
                               * 01010101010101......
                               * 
                               * (0 being off, 1 being blue)
                               */
                              for (int i = 1; i < m_LEDSubsystem.getBufferLength(); i += 2) { // Sets every other LED to
                                                                                              // bright blue
                                    m_buffer.setRGB(i, 0, 0, 255);
                              }

                              // Sets animCheck to true for pattern switch next time the delay is finished
                              animCheck = true;

                        }

                  }

                  // RED ALLIANCE
                  if (allianceColor == Alliance.Red) {

                        if (animCheck == true) {

                              /**
                               * sets the LED pattern to:
                               * 10101010101010......
                               * 
                               * (0 being off, 1 being red)
                               */
                              for (int i = 0; i < m_LEDSubsystem.getBufferLength(); i++) {
                                    if (i % 2 == 0) { // Sets every other LED to bright red
                                          m_buffer.setRGB(i, 255, 0, 0);
                                    } else {
                                          m_buffer.setRGB(i, 150, 150, 150);
                                    }
                              }

                              // Sets animCheck to false for pattern switch next time the delay is finished
                              animCheck = false;

                        } else {

                              /**
                               * clears the LED strip and sets all LED's to off
                               */
                              for (int i = 0; i < m_LEDSubsystem.getBufferLength(); i++) {
                                    m_buffer.setRGB(i, 150, 150, 150);
                              }

                              /**
                               * Sets the LED pattern to:
                               * 01010101010101......
                               * 
                               * (0 being off, 1 being red)
                               */
                              for (int i = 1; i < m_LEDSubsystem.getBufferLength(); i += 2) { // Sets every other LED to
                                                                                              // bright red
                                    m_buffer.setRGB(i, 255, 0, 0);
                              }

                              // Sets animCheck to true for pattern switch next time the delay is finished
                              animCheck = true;

                        }

                  }
                  m_LEDSubsystem.setBuffer(m_buffer);
                  // Resets the counter to 0 once the pattern is set
                  counter = 0;
            }
      }

      public void SolidLEDs() {
            counter =0;
            for (int i = 0; i < m_LEDSubsystem.getBufferLength(); i++) {
                  if(counter%5 == 0){
                        for (i = 0; i < m_LEDSubsystem.getBufferLength(); i++) {
                              m_buffer.setRGB(i, 0, 0, 0);
                        }
                  }
                  else{
                         for (i = 0; i < m_LEDSubsystem.getBufferLength(); i++) {
                              m_buffer.setRGB(i, 225, 140, 0);
                        }
                  }
                  counter++;
            }
            m_LEDSubsystem.setBuffer(m_buffer);
      }


}