#include "SamBusDriver.h"

#include "../../SafeOff.h"
#include "../PioAllocation.h"
#include "SamBusPins.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/pio.h"
#include "pico/time.h"
#include "sam_bus.pio.h"

namespace sambus {

namespace {

alignas(4) uint32_t g_frames[2][kFrameWords];
alignas(4) uint32_t g_safe[kFrameWords];
// Read by the reload channel at the end of every frame, then pointed back at
// the safe frame by the reset channel.
const uint32_t* volatile g_next = g_safe;
const uint32_t* const g_safePtr = g_safe;

PioSlot g_slot;
int g_data = -1, g_reload = -1, g_reset = -1;
uint8_t g_buildIndex = 0;
Driver* g_driver = nullptr;

void onDmaIrq() {
  if (g_data >= 0 && dma_channel_get_irq1_status(static_cast<uint>(g_data))) {
    dma_channel_acknowledge_irq1(static_cast<uint>(g_data));
    if (g_driver) g_driver->onFrameDone();
  }
}

}  // namespace

void Driver::onFrameDone() {
  // Drain STATUS bytes (one read per frame, on line 0).
  while (!pio_sm_is_rx_fifo_empty(g_slot.pio, g_slot.sm)) {
    status_ = static_cast<uint8_t>(pio_sm_get(g_slot.pio, g_slot.sm) & 0xFF);
  }
  // The board watchdog holds the outputs off: leave the safe frame queued
  // (strobes running, every output off) and drop what the device layer had
  // set, so nothing comes back on until it is written again, as a GPIO output
  // only comes back on its next analogWrite().
  if (g_outputsForcedOff) {
    frame_.allOff();
    return;
  }
  // The frame queued last time is playing now; build into the other one.
  uint32_t* buf = g_frames[g_buildIndex];
  frame_.build(buf);
  g_next = buf;
  g_buildIndex ^= 1;
  framesBuilt_ = framesBuilt_ + 1;
}

bool Driver::begin() {
  if (running_) return true;
  g_driver = this;

  // IO board in reset and J1 buffers off while the bus comes up (both lines
  // are pulled that way on the board too).
  gpio_init(SAM_BUS_PIN_NBRESET);
  gpio_put(SAM_BUS_PIN_NBRESET, 1);
  gpio_set_dir(SAM_BUS_PIN_NBRESET, GPIO_OUT);
  gpio_init(SAM_BUS_PIN_OE_N);
  gpio_put(SAM_BUS_PIN_OE_N, 1);
  gpio_set_dir(SAM_BUS_PIN_OE_N, GPIO_OUT);

  g_slot.program = &sam_bus_program;
  if (!pioClaimSlots(&g_slot, 1)) return false;
  PIO pio = g_slot.pio;
  const uint sm = g_slot.sm;

  pio_sm_config c = sam_bus_program_get_default_config(g_slot.offset);
  sm_config_set_out_pins(&c, SAM_BUS_PIN_D0, 12);
  sm_config_set_in_pins(&c, SAM_BUS_PIN_D0);
  sm_config_set_sideset_pins(&c, SAM_BUS_PIN_IOSTB);
  sm_config_set_out_shift(&c, true, false, 32);  // command word LSB first
  sm_config_set_in_shift(&c, false, false, 32);  // read byte ends in bits 7:0
  // 35 ns per cycle, whatever the system clock (SamBusFrame counts cycles).
  sm_config_set_clkdiv(&c, static_cast<float>(clock_get_hz(clk_sys)) / (1e9f / kCycleNs));
  for (uint pin = SAM_BUS_PIN_D0; pin <= SAM_BUS_PIN_DIR; ++pin) pio_gpio_init(pio, pin);
  const uint32_t mask = 0x3FFFu << SAM_BUS_PIN_D0;
  pio_sm_set_pins_with_mask(pio, sm, (3u << SAM_BUS_PIN_IOSTB), mask);  // IOSTB high, DIR drive
  pio_sm_set_pindirs_with_mask(pio, sm, mask, mask);
  pio_sm_init(pio, sm, g_slot.offset, &c);

  g_data = dma_claim_unused_channel(false);
  g_reload = dma_claim_unused_channel(false);
  g_reset = dma_claim_unused_channel(false);
  if (g_data < 0 || g_reload < 0 || g_reset < 0) return false;

  frame_.buildSafe(g_safe);
  g_next = g_safe;

  // data: one frame into the PIO, then hand over to reload.
  dma_channel_config d = dma_channel_get_default_config(static_cast<uint>(g_data));
  channel_config_set_transfer_data_size(&d, DMA_SIZE_32);
  channel_config_set_read_increment(&d, true);
  channel_config_set_write_increment(&d, false);
  channel_config_set_dreq(&d, pio_get_dreq(pio, sm, true));
  channel_config_set_chain_to(&d, static_cast<uint>(g_reload));
  dma_channel_configure(static_cast<uint>(g_data), &d, &pio->txf[sm], g_safe, kFrameWords, false);

  // reload: start the frame g_next points to, then hand over to reset.
  dma_channel_config r = dma_channel_get_default_config(static_cast<uint>(g_reload));
  channel_config_set_transfer_data_size(&r, DMA_SIZE_32);
  channel_config_set_read_increment(&r, false);
  channel_config_set_write_increment(&r, false);
  channel_config_set_chain_to(&r, static_cast<uint>(g_reset));
  dma_channel_configure(static_cast<uint>(g_reload), &r,
                        &dma_hw->ch[g_data].al3_read_addr_trig, &g_next, 1, false);

  // reset: point g_next back at the safe frame. The interrupt must queue a
  // fresh frame before the next one starts, or the safe frame plays.
  dma_channel_config z = dma_channel_get_default_config(static_cast<uint>(g_reset));
  channel_config_set_transfer_data_size(&z, DMA_SIZE_32);
  channel_config_set_read_increment(&z, false);
  channel_config_set_write_increment(&z, false);
  dma_channel_configure(static_cast<uint>(g_reset), &z, &g_next, &g_safePtr, 1, false);

  dma_channel_set_irq1_enabled(static_cast<uint>(g_data), true);
  irq_add_shared_handler(DMA_IRQ_1, onDmaIrq, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
  irq_set_enabled(DMA_IRQ_1, true);

  pio_sm_set_enabled(pio, sm, true);
  gpio_put(SAM_BUS_PIN_OE_N, 0);
  dma_channel_start(static_cast<uint>(g_reload));  // loads g_next and starts data

  // Let the lamp strobes run for a few frames before the IO board leaves
  // reset, so its watchdog sees DRV0 toggling from its first moment.
  sleep_us(4 * frame_.frameUs());
  gpio_put(SAM_BUS_PIN_NBRESET, 0);
  running_ = true;
  return true;
}

}  // namespace sambus
