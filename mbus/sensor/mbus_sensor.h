#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/mbus/mbus.h"

namespace esphome {
namespace mbus {

class MbusSensor : public sensor::Sensor, public Component {
 public:
  void set_parent(Mbus* parent) { parent_ = parent; }
  void set_mbus_storage(uint64_t mbus_storage) { mbus_storage_requested_ = mbus_storage; }
  void set_mbus_function(enum MbusDIFFunction mbus_function) { mbus_function_requested_ = mbus_function; }
  void set_mbus_tariff(uint32_t mbus_tariff) { mbus_tariff_requested_ = mbus_tariff; }
  void set_mbus_subunit(uint32_t mbus_subunit) { mbus_subunit_requested_ = mbus_subunit; }
  void set_mbus_vife(uint64_t mbus_vife) { mbus_vif_vife_requested_ = mbus_vife; }
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override;

 protected:
  Mbus* parent_;
  uint8_t telegram_seen;

  uint64_t mbus_storage_requested_;
  enum MbusDIFFunction mbus_function_requested_;
  uint32_t mbus_tariff_requested_;
  uint32_t mbus_subunit_requested_;
  uint64_t mbus_vif_vife_requested_;
  
};

}  // namespace mbus
}  // namespace esphome
