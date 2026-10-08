/*
  SamBusPins.h
  Created for the SAM_IO board, 2026.

  GPIOs of the standalone SAM_IO board (Ashram56/Stern-SAM-CPU-PPUC,
  hardware/sam_io_board, derived from IO_16_8_1). Its J9 connects to the SAM
  IO board's J1 with a straight 20-way ribbon. RS485 keeps GPIO 0-2, the LED
  GPIO 25, the board-id ladder GPIO 28 and the special output GPIO 29, as on
  IO_16_8_1. D0-D7 and A0-A3 must be consecutive, followed by IOSTB and DIR,
  for sam_bus.pio.

  The buffers between these pins and J9 (from the schematic):
    D0-D7     SN74LVC8T245 U7, DIR high = toward J9, 10 k pull-down on DIR
    A0-A3     74AHCT541 U8, IOSTB through U8 as well
    OE_N      enables U7 and U8, 10 k pull-up: off until the firmware starts
    NBRESET   2N7002 Q1 open drain on J9 pin 13, 10 k gate pull-up: the IO
              board stays in reset from power-up until this pin goes low,
              and goes back into reset if the RP2040 resets

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
#define SAM_BUS_PIN_OE_N 17    // low = J9 buffers enabled (pulled up)
#endif
#ifndef SAM_BUS_PIN_NBRESET
#define SAM_BUS_PIN_NBRESET 18 // high = IO board held in reset (pulled up)
#endif

static_assert(SAM_BUS_PIN_IOSTB == SAM_BUS_PIN_D0 + 12 &&
                  SAM_BUS_PIN_DIR == SAM_BUS_PIN_IOSTB + 1,
              "sam_bus.pio needs D0-D7, A0-A3, IOSTB, DIR on consecutive GPIOs");

#endif
