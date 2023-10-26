#include "bl0942.h"
#include "esphome/core/log.h"

namespace esphome {
namespace bl0942 {
static const char *const TAG = "bl0942";

static const uint8_t BL0942_READ_COMMAND = 0x58;
static const uint8_t BL0942_FULL_PACKET = 0xAA;
static const uint8_t BL0942_PACKET_HEADER = 0x55;

static const uint8_t BL0942_WRITE_COMMAND = 0xA8;
static const uint8_t BL0942_REG_I_FAST_RMS_CTRL = 0x10;
static const uint8_t BL0942_REG_MODE = 0x18;
static const uint8_t BL0942_REG_SOFT_RESET = 0x19;
static const uint8_t BL0942_REG_USR_WRPROT = 0x1A;
static const uint8_t BL0942_REG_TPS_CTRL = 0x1B;

// TODO: Confirm insialisation works as intended
const uint8_t BL0942_INIT[5][6] = {
    // Reset to default
    {BL0942_WRITE_COMMAND, BL0942_REG_SOFT_RESET, 0x5A, 0x5A, 0x5A, 0x38},
    // Enable User Operation Write
    {BL0942_WRITE_COMMAND, BL0942_REG_USR_WRPROT, 0x55, 0x00, 0x00, 0xF0},
    // 0x0100 = CF_UNABLE energy pulse, AC_FREQ_SEL 50Hz, RMS_UPDATE_SEL 800mS
    {BL0942_WRITE_COMMAND, BL0942_REG_MODE, 0x00, 0x10, 0x00, 0x37},
    // 0x47FF = Over-current and leakage alarm on, Automatic temperature measurement, Interval 100mS
    {BL0942_WRITE_COMMAND, BL0942_REG_TPS_CTRL, 0xFF, 0x47, 0x00, 0xFE},
    // 0x181C = Half cycle, Fast RMS threshold 6172
    {BL0942_WRITE_COMMAND, BL0942_REG_I_FAST_RMS_CTRL, 0x1C, 0x18, 0x00, 0x1B}};

void BL0942::loop() {
  if (this->available()) {
    while (this->available()) {
       uint8_t in;
       read_array(&in,1);
       if(inpos<sizeof(buffer)-1){ // читаем тело пакета
          ((uint8_t*)(&buffer))[inpos]=in;
          inpos++;
          checksum += in;
       } else if(inpos<sizeof(buffer)){ // получили контрольную сумму
          inpos++;
          checksum^=0xFF;
          if(in!=checksum){
             ESP_LOGE(TAG, "BL0942 invalid checksum! 0x%02X != 0x%02X", checksum, in);
          } else {
             pubPhase=0;
          }
       } else {
          if(in==BL0942_PACKET_HEADER){ // стартовый хидер
             ((uint8_t*)(&buffer))[0]=BL0942_PACKET_HEADER;
             inpos=1; // начало сохранения буфера  
             checksum=BL0942_READ_COMMAND+BL0942_PACKET_HEADER; //начальные данные рассчета кс
             pubPhase=3;
          } else {
             ESP_LOGE(TAG, "Invalid data. Header mismatch: %d", in);
          }
       }
    }
  } else if(pubPhase<3){
     if(pubPhase==0){ 
        if (current_sensor_ != nullptr) {
          float i_rms = (uint24_t) buffer.i_rms / current_reference_;
          current_sensor_->publish_state(i_rms);
        }
        if (power_sensor_ != nullptr) {
           float watt = (int24_t) buffer.watt / power_reference_;
           power_sensor_->publish_state(watt);
        }
        pubPhase=1;
     } else if(pubPhase==1){ 
        if (voltage_sensor_ != nullptr) {
          float v_rms = (uint24_t) buffer.v_rms / voltage_reference_;
          voltage_sensor_->publish_state(v_rms);
        }
        if (frequency_sensor_ != nullptr) {
           float frequency = 1000000.0f / buffer.frequency;
           frequency_sensor_->publish_state(frequency);
        }
        pubPhase=2;
     } else if(pubPhase==2){ 
        if (energy_sensor_ != nullptr) {
           uint32_t cf_cnt = (uint24_t) buffer.cf_cnt;
           float total_energy_consumption = cf_cnt / energy_reference_;
           energy_sensor_->publish_state(total_energy_consumption);
        }
        pubPhase=3;
     }
  } if(needUpdate){
     this->write_byte(BL0942_READ_COMMAND);
     this->write_byte(BL0942_FULL_PACKET);
     needUpdate=false;
  }
}

void BL0942::update() {
  needUpdate=true;
}

void BL0942::setup() {
  for (auto *i : BL0942_INIT) {
    this->write_array(i, 6);
    delay(1);
  }
  this->flush();
}

void BL0942::dump_config() {  // NOLINT(readability-function-cognitive-complexity)
  ESP_LOGCONFIG(TAG, "BL0942:");
  LOG_SENSOR("", "Voltage", this->voltage_sensor_);
  LOG_SENSOR("", "Current", this->current_sensor_);
  LOG_SENSOR("", "Power", this->power_sensor_);
  LOG_SENSOR("", "Energy", this->energy_sensor_);
  LOG_SENSOR("", "frequency", this->frequency_sensor_);
}

}  // namespace bl0942
}  // namespace esphome