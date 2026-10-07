# SAM_IO: what it adds to io-boards

SAM_IO is the stock PPUC io-boards firmware plus one output back end: coils,
flashers and lamps are written to an original Stern SAM IO power driver board
over its CPU bus (J1) instead of to GPIO PWM. Everything else (protocol, event
dispatcher, PwmDevices pulse logic and fast-flip safety, config handling,
watchdog) is upstream code, unchanged.

To review SAM_IO, read this folder, then the touch points below. Nothing
else differs from upstream.

## This folder (new)

| File | What |
|---|---|
| `SamIoBoard.h` | the only object `IOBoardController` knows about: starts the bus, plugs `SamBusOutput` into `PwmDevices` |
| `SamBusOutput.h` | `PwmOutput` that turns `PwmDevices` writes into SAM coil and lamp levels |
| `SamBusMap.h` | SAM IO board registers and the PPUC port numbering |
| `SamBusFrame.h/.cpp` | builds one lamp frame of bus commands: lamp scan, coil PWM, lamp brightness and ramps, aux boards. Pure C++, tested by `test/test_sam_bus` |
| `SamBusDriver.h/.cpp`, `sam_bus.pio` | plays frames on PIO with chained DMA; all-off frame when late or while the watchdog holds outputs off |
| `SamBusPins.h` | GPIO map of the SAM_IO board |

## Touch points in upstream files

| File | Change |
|---|---|
| `src/IODevices/PwmOutput.h` (new) | output interface; `GpioPwmOutput` is the old `pinMode`/`analogWrite` behaviour |
| `src/IODevices/PwmDevices.h/.cpp` | `analogWrite(port[i], x)` becomes `writeOutput(i, x)`, plus `setOutput()`. Default output is GPIO, so other boards behave as before |
| `src/IOBoardController.h/.cpp` | one `SamIoBoard*` member, created in `begin()` on boards with `kCapSamBus`; `registerPwmOutput()` checks ports with `allowsOutput()` and skips the PWM channel bookkeeping on SAM_IO |
| `src/PPUCBoardTypes.h` | `kCapSamBus`, the `kSamIo` profile, `Profile::allowsOutput()` (same as `allowsPwm()` on every other board) |
| `src/PPUCProtocolV2.h` | board type `0x05` `SAM_IO` |
| `platformio.ini`, `.github/workflows/io-boards.yml` | `SAM_IO` environment and CI build |
| `test/test_board_profiles`, `test/test_sam_bus` | tests |
