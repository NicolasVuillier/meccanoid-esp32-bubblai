# Meccano M.A.X. – experimental two-servo LIM profile

**Not hardware-tested or compiled with the ESP32 toolchain yet.** Do not replace the working G15 sketch. Use an ESP32-WROOM DevKit with Arduino ESP32 3.x (the G15 project uses 3.3.11), and open `MeccanoidMAXLim.ino` with its adjacent driver files.

## Wiring
- Original M.A.X. servo chain: head first (ID 0), claw second (ID 1). Data on GPIO25; leave face, wheels and IR disconnected for this first test.
- Supply both servos from a regulated **5 V** external source with sufficient current; feed the ESP32 through its USB connector, with common ground. Do not route servo current through the ESP32. Original M.A.X. battery is nominally 6 V, but the official 2015 Smart Module protocol lists 5 V on the red wire.
- The data line is bidirectional; confirm both directions are safe at 3.3 V and use an appropriate bidirectional level interface if required. Never apply 5 or 6 V to the GPIO. Identify the actual pins and cut power before rewiring.
- Keep an accessible hardware power switch, no object or fingers in the claw, wheels disconnected. On initial testing, remove mechanical load as practical.

## First tests
1. Start the ESP32 and send `[DIAGNOSTIC]` in USB serial (9600 baud). Wait for both servo positions to become valid.
2. Send `[LIM]` and check, without forcing, that head and claw move freely and `[POSITIONS]` changes. The MAX report specifies 0xF9; this separate driver defaults to it. If it is not released, stop and investigate (the original G15 implementation uses 0xFA; an alternate compile-time MAX_LIM_COMMAND is available only for controlled comparison).
3. Use `[ENREGISTRE_TEST]`, teach a very small safe movement, and send `[FIN_ENREGISTREMENT]`. `[LISTE_MOUVEMENTS]` should show TEST. `[TEST]` replays it with a gradual approach and limited command step. `[STOP]` releases both servos; hardware cutoff is still required if communication fails.
4. If the claw passes testing, manually teach `OUVRE_PINCE` and `FERME_PINCE`; use those names as voice actions. No fixed open/closed angles or pinch force are assumed. A learned motion is not a force controller.
5. BLE advertises as Mecanoid, same UART service/RX/TX as G15. Send complete commands, with or without brackets; use one movement per message. Keep only one robot advertising during setup.

Recordings are separate MAX_*.bin files, not compatible with G15 trajectories. This sketch refuses to format LittleFS on failure; use a fresh/dedicated MAX flash partition, initialize it if necessary, and never erase a G15 partition accidentally. It saves both axes only on fresh feedback and replays only axes which moved at least 4 degrees. It checks for stale feedback and releases on BLE disconnect or queue overflow. Step limiting may extend or smooth timing. STOP while recording attempts to save; LIM discards an active capture. There is no automatic boot movement. USB is the diagnostic output.

## Next phase: face
The M.A.X. has a separate 16×8 monochrome matrix, module type 0x06, not the RGB Smart LED from the G15. Its bitstream has distinct pulse timings (~140 us sub-symbols), an 18-byte frame (16 display columns, flag 0xF5, checksum), and a roughly 62 ms transmission. A dedicated GPIO and an ESP32 hardware-timed transmitter (e.g. RMT) are recommended; verify bit orientation/polarity on hardware. Map existing BubblAI emotion names to bitmaps and update only when needed. No display commands are implemented in this branch. Motors and IR also remain separate future work.

## References
- https://cdn.meccano.com/open-source/Meccano_SmartModuleProtocols_2015.pdf
- https://forum.arduino.cc/t/meccano-max-and-arduino/533601
- https://cdck-file-uploads-europe1.s3.dualstack.eu-west-1.amazonaws.com/arduino/original/3X/2/3/23cd781219fb354c963e4d939cd619e206bf6e03.pdf
- https://hackaday.io/project/160606-hacking-the-meccano-max/
