#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include <vector>

namespace esphome {
namespace mbus {
	
static const uint8_t mbus_max_retries_ = 3;

static const unsigned char* mbus_reset_frame_ = (const unsigned char*)"\x10\x40\xFD\x3D\x16";
static const size_t mbus_reset_frame_len_ = 5;

static const unsigned char* mbus_request_frame_ = (const unsigned char*)"\x10\x5B\xFD\x58\x16";
static const size_t mbus_request_frame_len_ = 5;

static const unsigned char* mbus_select_frame_raw_ = 
	(const unsigned char*)"\x68\x0B\x0B\x68\x73\xFD\x52\x00\x00\x00\x00\x00\x00\x00\x00\x00\x16";
static const size_t mbus_select_frame_len_ = 17;

static const uint8_t mbus_ack_ = 0xE5;
static const uint8_t mbus_long_frame_ = 0x68;

static const uint8_t MBUS_CONTROL_RSP_UD = 0X08;
static const uint8_t MBUS_CI_RESP_VARIABLE = 0x72;
static const uint8_t MBUS_STATUS_APP_ERROR = 0x03;
static const uint8_t MBUS_STATUS_LOW_POWER = 0x04;
static const uint8_t MBUS_STATUS_PERMANENT_ERROR = 0x08;
static const uint8_t MBUS_STATUS_TEMPORARY_ERROR = 0x10;
static const uint8_t MBUS_DIF_MANUFACTURER_SPECIFIC = 0x0F;
static const uint8_t MBUS_DIF_MANUFACTURER_SPECIFIC_MULTIFRAME = 0x1F;
static const uint8_t MBUS_DIF_FILLER = 0x2F;

enum MbusDIFDatatype : uint8_t {
  MBUS_NO_DATA = 0X00,
  MBUS_INT_8BIT = 0X01,
  MBUS_INT_16BIT = 0X02,
  MBUS_INT_24BIT = 0X03,
  MBUS_INT_32BIT = 0X04,
  MBUS_INT_48BIT = 0X06,
  MBUS_INT_64BIT = 0X07,
  MBUS_REAL = 0X05,
  MBUS_SELECTION = 0X08,
  MBUS_BCD2 = 0X09,
  MBUS_BCD4 = 0X0A,
  MBUS_BCD6 = 0X0B,
  MBUS_BCD8 = 0X0C,
  MBUS_VARIABLE_LEN = 0X0D,
  MBUS_BCD12 = 0X0E,
  MBUS_SPECIAL = 0X0F,
};

enum MbusDIFFunction : uint8_t {
  MBUS_INSTANT_VALUE = 0X00,
  MBUS_MAXIMUM_VALUE = 0X10,
  MBUS_MINIMUM_VALUE = 0X20,
  MBUS_ERROR_VALUE = 0X30,
};

const char* MbusDIFDatatypeToStr(enum MbusDIFDatatype datatype);
const char* MbusDIFFunctionToStr(enum MbusDIFFunction function);

enum MbusState {
	MBUS_STATE_IDLE,
	MBUS_STATE_AWAIT_LOCK,
	MBUS_STATE_BUS_RESET_PRE,
	MBUS_STATE_BUS_RESET,
	MBUS_STATE_BUS_RESET_2,
	MBUS_STATE_AWAIT_SELSCT_SA,
	MBUS_STATE_AWAIT_SELSCT_SA_2,
	MBUS_STATE_AWAIT_HEADER,
	MBUS_STATE_AWAIT_DATA,
	MBUS_STATE_RETRY_WAIT,
	MBUS_STATE_RETRY,
}; 

// One decoded Data Record from the telegram. The hub decodes every record
// exactly once per telegram and caches them here; MbusSensor instances just
// look up their own (function, storage, tariff, subunit, vif_vife) in this
// list instead of re-parsing the raw bytes themselves.
struct MbusRecord {
  enum MbusDIFFunction function;
  uint64_t storage;
  uint32_t tariff;
  uint16_t subunit;
  uint64_t vif_vife;
  float value;
};
	
class Mbus : public uart::UARTDevice, public PollingComponent {
 public:

  void setup() override;

  void loop() override;

  void dump_config() override;
  
  void update() override;

  float get_setup_priority() const override;
  
  void set_secondary_address(uint64_t secondary_address) { this->secondary_address = secondary_address; }
  
  uint64_t secondary_address;
  
  uint8_t telegram[270];
  uint8_t telegram_count;

  // Decoded Data Records of the most recently received telegram. Populated
  // once by parse_telegram() whenever telegram_count is incremented.
  std::vector<MbusRecord> records;

 protected:
 
 
 uint32_t mbus_timeout_short_;
 uint32_t mbus_timeout_long_;
 uint32_t mbus_timer_;
 enum MbusState mbus_state_;
 uint8_t mbus_retry_count_;
 bool mbus_update_due_;
 uint16_t mbus_telegram_len_;
 uint8_t mbus_select_frame_[mbus_select_frame_len_];
  
  uint8_t mbus_checksum(const uint8_t* data);

  // Parses the fixed header and all variable-length Data Records of the
  // just-received telegram (this->telegram) into this->records. Runs once
  // per new telegram - this used to be duplicated in every MbusSensor::loop().
  void parse_telegram();

  // Parses a single Data Record starting at tg (pointing at its DIF).
  // Returns the number of bytes consumed, or 0 on error (logs the error).
  uint8_t parse_data_record(uint8_t* tg, MbusRecord& record_out);
  
};

	
}  // namespace mbus
}  // namespace esphome
