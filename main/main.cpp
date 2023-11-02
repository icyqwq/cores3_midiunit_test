#include <M5Unified.h>

HardwareSerial &MIDISerial = Serial1;

static void app_midi_write(uint8_t channel, uint8_t command, uint8_t arg1, uint8_t arg2 = 0)
{
    if (command < 128)
    {
        // shift over command
        command <<= 4;
    }
    command |= channel;
    char buf[3] = {command, arg1, arg2};

    // send MIDI data
    MIDISerial.write(buf, 3);
}

static void app_midi_test()
{
    uint8_t notes[] = {60, 62, 64, 65, 67, 69, 71, 72}; // C4, D4, E4, F4, G4, A4, B4, C5

    for (int i = 0; i < 8; i++)
    {
        app_midi_write(0, 0x90, notes[i], 0x7f);
        vTaskDelay(100);
        app_midi_write(0, 0x80, notes[i], 0x45);
    }
}

void setup(void)
{
    M5.begin();

    M5.Display.setFont(&fonts::Font4);
    M5.Display.drawCenterString("CoreS3 Midi Unit Test", 160, 60);

    MIDISerial.begin(31250, SERIAL_8N1, -1, 2); // connect to unit rx.
}

void loop(void)
{
    printf("write\n");
    app_midi_test();
    vTaskDelay(1000);
}
