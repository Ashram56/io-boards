/*
  SamBusPins.h
  Created for the SAM_IO board, 2026.

  GPIOs of the standalone SAM_IO board (an RP2040 on RS485, plugged into the
  SAM IO board's J1). RS485 keeps GPIO 0-2 and the board-id ladder GPIO 28,
  as on every PPUC board. D0-D7 and A0-A3 must be consecutive, followed by
  IOSTB and DIR, for sam_bus.pio.

  Play more pinball!
*/
#ifndef SAMBUS_SamBusPins_h
#define SAMBUS_SamBusPins_h

#ifndef SAM_BUS_PIN_D0
#define SAM_BUS_PIN_D0 3       // D0-D7 = GPIO 3-10, A0-A3 = GPIO 11-14
#endif
#ifndef SAM_BUS_PIN_IOSTB
#define SAM_BUS_PIN_IOSTB 15   // side-set bit 0, active low
#endif
#ifndef SAM_BUS_PIN_DIR
#define SAM_BUS_PIN_DIR 16     // side-set bit 1, 1 = this board drives J1
#endif
#ifndef SAM_BUS_PIN_OE_N
#define SAM_BUS_PIN_OE_N 17    // low = J1 buffers enabled (pulled up)
#endif
#ifndef SAM_BUS_PIN_NBRESET
#define SAM_BUS_PIN_NBRESET 18 // high = IO board held in reset (pulled up)
#endif

static_assert(SAM_BUS_PIN_IOSTB == SAM_BUS_PIN_D0 + 12 &&
                  SAM_BUS_PIN_DIR == SAM_BUS_PIN_IOSTB + 1,
              "sam_bus.pio needs D0-D7, A0-A3, IOSTB, DIR on consecutive GPIOs");

#endif
