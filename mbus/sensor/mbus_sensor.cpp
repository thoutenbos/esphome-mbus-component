#include "mbus_sensor.h"

#include "esphome/core/log.h"

namespace esphome {
namespace mbus {

static const char *const TAG = "mbus.sensor";

void MbusSensor::setup() {
  this->telegram_seen = 0;
}

float MbusSensor::get_setup_priority() const {   return setup_priority::BUS - 1.0f; }
void MbusSensor::dump_config() {
  LOG_SENSOR("", "Mbus Sensor", this);
  ESP_LOGCONFIG(TAG, "  Secondary address: %llX" , this->parent_->secondary_address);
  ESP_LOGCONFIG(TAG, "  Storage number: %lld" , this->mbus_storage_requested_);
  ESP_LOGCONFIG(TAG, "  Function: %s" , MbusDIFFunctionToStr(this->mbus_function_requested_));
  ESP_LOGCONFIG(TAG, "  Tariff: %d" , this->mbus_tariff_requested_);
  ESP_LOGCONFIG(TAG, "  Subunit: %d" , this->mbus_subunit_requested_);
  ESP_LOGCONFIG(TAG, "  VIF/VIFE: 0x%llX" , this->mbus_vif_vife_requested_);
}

void MbusSensor::loop() {

  //check if the hub has parsed a new telegram since we last looked
  if(this->telegram_seen == this->parent_->telegram_count)return;
  this->telegram_seen = this->parent_->telegram_count;

  const char* sensorname=this->get_name().c_str();

  //look up our own criteria in the hub's already-decoded records instead of
  //re-parsing the raw telegram ourselves - the hub only had to do that once,
  //for all sensors together.
  uint8_t matches = 0;
  float value = 0;

  for (auto &rec : this->parent_->records) {
    if( (rec.storage == this->mbus_storage_requested_) &&
        (rec.function == this->mbus_function_requested_) &&
        (rec.tariff == this->mbus_tariff_requested_) &&
        (rec.subunit == this->mbus_subunit_requested_) &&
        (rec.vif_vife == this->mbus_vif_vife_requested_) ) {
      matches++;
      value = rec.value;
    }
  }

  if(matches == 0){
    ESP_LOGE(TAG, " %s: Specified data record not in telegram", sensorname);
    return;
  }

  if(matches > 1){
    ESP_LOGE(TAG, " %s: Multiple matching data records in telegram", sensorname);
    return;
  }

  ESP_LOGD(TAG, " %s: Match", sensorname);
  ESP_LOGI(TAG, "%s: New raw value: %.1f", sensorname, value);
  this->publish_state(value);

}

}  // namespace mbus
}  // namespace esphome
