#include "transmitter.h"
#include "debug_logger.h"

#include <ELECHOUSE_CC1101_SRC_DRV.h>

namespace esphome {
namespace q7rf_tx {

class Transmitter::SendTask {
  uint8_t remaining_;
  uint8_t *act_;
  DebugLogger l_;
  uint32_t loops_;

  std::function<void(bool)> result_callback_;

  static uint8_t read_tx_bytes() {
    uint8_t data[1];
    ELECHOUSE_cc1101.SpiReadBurstReg(CC1101_TXBYTES, data, sizeof(data));
    return *data;
  }

  void send_(uint8_t can_send) {
    const uint8_t send = std::min(can_send, remaining_);
    ELECHOUSE_cc1101.SpiWriteBurstReg(CC1101_TXFIFO, act_, send);  // write data to send
    act_ += send;
    remaining_ -= send;
    l_ << "R_" << static_cast<int>(remaining_) << ' ';
  }

  void end_(bool success) {
    ELECHOUSE_cc1101.SpiStrobe(CC1101_SFTX);  // flush TXfifo
    l_ << "FLS" << ' ';
    l_ << "B_" << static_cast<int>(read_tx_bytes()) << ' ';

    ELECHOUSE_cc1101.setSidle();
    l_ << "IDL" << ' ';
    l_ << "B_" << static_cast<int>(read_tx_bytes()) << ' ';

    l_ << "LOOPS_" << static_cast<int>(loops_);

    if (result_callback_)
      result_callback_(success);
  }

  static const uint8_t FIFO_MAX = 64;

 public:
  SendTask(const std::vector<uint8_t> &data, const std::function<void(bool)> &result_callback)
      : remaining_(data.size()),
        act_(const_cast<uint8_t *>(data.data())),
        l_(DebugLogger::FLUSH_COMPLETE, TAG),
        loops_(0),
        result_callback_(result_callback) {}

  SendTask(const SendTask &) = delete;
  SendTask &operator=(const SendTask &) = delete;

  bool process() {
    if (loops_ == 0) {
      // ELECHOUSE_cc1101.setPacketLength(data.size ());

      l_ << "B_" << static_cast<int>(read_tx_bytes()) << ' ';
      send_(FIFO_MAX);
      l_ << "B_" << static_cast<int>(read_tx_bytes()) << ' ';

      ELECHOUSE_cc1101.SetTx();

      l_ << "SYNC" << ' ';
    }
    ++loops_;
    if (remaining_ > 0) {
      const uint8_t txbytes = read_tx_bytes();
      if (txbytes & 0x80) {
        ESP_LOGE(TAG, "Transfer failed txbytes: %x", txbytes);
        end_(false);
        return true;
      }
      if (txbytes < FIFO_MAX / 2) {
        l_ << "B_" << static_cast<int>(txbytes) << ' ';
        send_(FIFO_MAX - txbytes);
      }
      if (remaining_ == 0) {
        l_ << "ELOOP" << ' ';
      }
      return false;
    }

    const uint8_t tx_bytes = read_tx_bytes();
    l_ << "B_" << static_cast<int>(tx_bytes) << ' ';

    uint8_t marcstate;
    ELECHOUSE_cc1101.SpiReadBurstReg(CC1101_MARCSTATE, &marcstate, sizeof(marcstate));
    l_ << "MS_" << static_cast<int>(marcstate) << ' ';

    if (tx_bytes == 0 || tx_bytes >= 128) {
      l_ << "EPKT" << ' ';
      end_(true);
      return true;
    }
    // if (marcstate == 1) {
    //     end_ ();
    //     return true;
    // }
    return false;
  }
};

Transmitter::Transmitter(uint8_t clk_pin, uint8_t miso_pin, uint8_t mosi_pin, uint8_t cs_pin)
    : clk_pin_(clk_pin), miso_pin_(miso_pin), mosi_pin_(mosi_pin), cs_pin_(cs_pin), module_number_(0), spi_status_(0) {}

Transmitter::~Transmitter() = default;

void Transmitter::setup() {
  ELECHOUSE_cc1101.addSpiPin(clk_pin_, miso_pin_, mosi_pin_, cs_pin_, module_number_);
  // ELECHOUSE_cc1101.addGDO0 (gdo0_pin, module_number_);
  ELECHOUSE_cc1101.setModul(module_number_);

  if (ELECHOUSE_cc1101.SpiReadStatus(0x31) > 0) {  // Check the CC1101 Spi connection.
    ESP_LOGV(TAG, "Connection OK");
    spi_status_ = 1;
  } else {
    ESP_LOGV(TAG, "Connection Error");
    spi_status_ = -1;
  }

  ELECHOUSE_cc1101.Init();            // must be set to initialize the cc1101!
  ELECHOUSE_cc1101.setCCMode(1);      // set config for internal transmission mode.
  ELECHOUSE_cc1101.setModulation(2);  // set modulation mode. 0 = 2-FSK, 1 = GFSK, 2 = ASK/OOK, 3 = 4-FSK, 4 = MSK.
  ELECHOUSE_cc1101.setMHZ(
      868.35);  // Here you can set your basic frequency. The lib calculates the frequency automatically (default =
                // 433.92).The cc1101 can: 300-348 MHZ, 387-464MHZ and 779-928MHZ. Read More info from datasheet.
  ELECHOUSE_cc1101.setDeviation(
      47.60);  // Set the Frequency deviation in kHz. Value from 1.58 to 380.85. Default is 47.60 kHz.
  ELECHOUSE_cc1101.setChannel(0);  // Set the Channelnumber from 0 to 255. Default is cahnnel 0.
  // ELECHOUSE_cc1101.setChsp(199.95);       // The channel spacing is multiplied by the channel number CHAN and added
  // to the base frequency in kHz. Value from 25.39 to 405.45. Default is 199.95 kHz. ELECHOUSE_cc1101.setRxBW(812.50);
  // // Set the Receive Bandwidth in kHz. Value from 58.03 to 812.50. Default is 812.50 kHz.
  ELECHOUSE_cc1101.setDRate(4.5);  // Set the Data Rate in kBaud. Value from 0.02 to 1621.83. Default is 99.97 kBaud!
  ELECHOUSE_cc1101.setPA(12);  // Set TxPower. The following settings are possible depending on the frequency band. (-30
                               // -20  -15  -10  -6    0    5    7    10   11   12) Default is max!
  ELECHOUSE_cc1101.setSyncMode(
      0);  // Combined sync-word qualifier mode. 0 = No preamble/sync. 1 = 16 sync word bits detected. 2 = 16/16 sync
           // word bits detected. 3 = 30/32 sync word bits detected. 4 = No preamble/sync, carrier-sense above
           // threshold. 5 = 15/16 + carrier-sense above threshold. 6 = 16/16 + carrier-sense above threshold. 7 = 30/32
           // + carrier-sense above threshold.
  // ELECHOUSE_cc1101.setSyncWord(211, 145); // Set sync word. Must be the same for the transmitter and receiver.
  // (Syncword high, Syncword low)
  ELECHOUSE_cc1101.setAdrChk(0);  // Controls address check configuration of received packages. 0 = No address check. 1
                                  // = Address check, no broadcast. 2 = Address check and 0 (0x00) broadcast. 3 =
                                  // Address check and 0 (0x00) and 255 (0xFF) broadcast.
  // ELECHOUSE_cc1101.setAddr(0);            // Address used for packet filtration. Optional broadcast addresses are 0
  // (0x00) and 255 (0xFF).
  ELECHOUSE_cc1101.setWhiteData(0);  // Turn data whitening on / off. 0 = Whitening off. 1 = Whitening on.
  ELECHOUSE_cc1101.setPktFormat(
      0);  // Format of RX and TX data. 0 = Normal mode, use FIFOs for RX and TX. 1 = Synchronous serial mode, Data in
           // on GDO0 and data out on either of the GDOx pins. 2 = Random TX mode; sends random data using PN9
           // generator. Used for test. Works as normal mode, setting 0 (00), in RX. 3 = Asynchronous serial mode, Data
           // in on GDO0 and data out on either of the GDOx pins.
  ELECHOUSE_cc1101.setLengthConfig(0);  // 0 = Fixed packet length mode. 1 = Variable packet length mode. 2 = Infinite
                                        // packet length mode. 3 = Reserved
  ELECHOUSE_cc1101.setPacketLength(
      255);  // Indicates the packet length when fixed packet length mode is enabled. If variable packet length mode is
             // used, this value indicates the maximum packet length allowed.
  ELECHOUSE_cc1101.setCrc(0);  // 1 = CRC calculation in TX and CRC check in RX enabled. 0 = CRC disabled for TX and RX.
  // ELECHOUSE_cc1101.setCRC_AF(0);          // Enable automatic flush of RX FIFO when CRC is not OK. This requires that
  // only one packet is in the RXIFIFO and that packet length is limited to the RX FIFO size.
  ELECHOUSE_cc1101.setDcFilterOff(0);  // Disable digital DC blocking filter before demodulator. Only for data rates ≤
                                       // 250 kBaud The recommended IF frequency changes when the DC blocking is
                                       // disabled. 1 = Disable (current optimized). 0 = Enable (better sensitivity).
  ELECHOUSE_cc1101.setManchester(0);   // Enables Manchester encoding/decoding. 0 = Disable. 1 = Enable.
  ELECHOUSE_cc1101.setFEC(0);  // Enable Forward Error Correction (FEC) with interleaving for packet payload (Only
                               // supported for fixed packet length mode. 0 = Disable. 1 = Enable.
  ELECHOUSE_cc1101.setPRE(0);  // Sets the minimum number of preamble bytes to be transmitted. Values: 0 : 2, 1 : 3, 2 :
                               // 4, 3 : 6, 4 : 8, 5 : 12, 6 : 16, 7 : 24
  ELECHOUSE_cc1101.setPQT(
      0);  // Preamble quality estimator threshold. The preamble quality estimator increases an internal counter by one
           // each time a bit is received that is different from the previous bit, and decreases the counter by 8 each
           // time a bit is received that is the same as the last bit. A threshold of 4∙PQT for this counter is used to
           // gate sync word detection. When PQT=0 a sync word is always accepted.
  ELECHOUSE_cc1101.setAppendStatus(0);  // When enabled, two status bytes will be appended to the payload of the packet.
                                        // The status bytes contain RSSI and LQI values, as well as CRC OK.

  ELECHOUSE_cc1101.setSidle();

  ESP_LOGV(TAG, "CC1101 module (%d) setup complete", module_number_);
}

void Transmitter::send(const std::vector<uint8_t> &data, const std::function<void(bool)> &result_callback) {
  tasks_.emplace(data, result_callback);
}

void Transmitter::loop() {
  if (!tasks_.empty()) {
    if (tasks_.front().process()) {
      tasks_.pop();
    }
  }
}

void Transmitter::dump_config() const {
  ESP_LOGCONFIG(TAG, "  CLK: %d, MISO: %d, MOSI: %d, CS: %d", clk_pin_, miso_pin_, mosi_pin_, cs_pin_);
  ESP_LOGCONFIG(TAG, "  SPI status:%d", spi_status_);
}

} // namespace q7rf_tx
} // namespace esphome
