#include "esppac_wlan.h"
#include "esppac_commands_wlan.h"

#include "esphome/core/log.h"

namespace esphome {
namespace panasonic_ac {
namespace WLAN {

static uint8_t determine_mode_value(climate::ClimateMode mode);

static const char *const TAG = "panasonic_ac.dnskp11";

void PanasonicACWLAN::setup() {
  PanasonicAC::setup();

  ESP_LOGD(TAG, "Using DNSK-P11 protocol via CN-WLAN");
}

void PanasonicACWLAN::loop() {
  if (this->state_ != ACState::Ready) {
    handle_init_packets();  // Handle initialization packets separate from normal packets

    if (millis() - this->init_time_ > INIT_FAIL_TIMEOUT) {
      this->state_ = ACState::Failed;
      mark_failed();
      return;
    }
  }

  // Serialise set packets: the protocol is strictly request/response, and sending
  // a new set while the previous one is still unacked desynchronises the packet
  // counters (logged as "Correcting shifted tx counter") and can drop commands.
  // Measured: a burst of 8 commands from HA lost the last one.
  if (this->pending_set_send_ && this->state_ == ACState::Ready && !this->waiting_for_response_) {
    this->pending_set_send_ = false;
    send_set_command();
  }

  if (millis() - this->last_read_ > READ_TIMEOUT &&
      !this->rx_buffer_.empty())  // Check if our read timed out and we received something
  {
    log_packet(this->rx_buffer_);

    // Check for defrost status packet
    if (this->rx_buffer_[0] == 0x70 && this->rx_buffer_.size() >= 15) {
      bool defrost = (this->rx_buffer_[14] == 0x02);
      update_defrost(defrost);
      this->rx_buffer_.clear();
      return;
    }

    if (!verify_packet())  // Verify length, header, counter and checksum
      return;

    this->waiting_for_response_ =
        false;  // Set that we are not waiting for a response anymore since we received a valid one
    this->last_packet_received_ = millis();  // Set the time at which we received our last packet

    if (this->state_ == ACState::Ready || this->state_ == ACState::FirstPoll ||
        this->state_ == ACState::HandshakeEnding)  // Parse regular packets
    {
      handle_packet();  // Handle regular packet
    } else              // Parse handshake packets
    {
      handle_handshake_packet();  // Not initialized yet, handle handshake packet
    }

    this->rx_buffer_.clear();  // Reset buffer
  }

  PanasonicAC::read_data();

  handle_resend();  // Handle packets that need to be resent

  handle_poll();  // Handle sending poll packets
}

/*
 * ESPHome control request
 */

void PanasonicACWLAN::control(const climate::ClimateCall &call) {
  if (this->state_ != ACState::Ready)
    return;

  if (call.get_mode().has_value()) {
    ESP_LOGV(TAG, "Requested mode change");

    switch (*call.get_mode()) {
      case climate::CLIMATE_MODE_COOL:
        set_value(0xB0, 0x42);
        set_value(0x80, 0x30);
        break;
      case climate::CLIMATE_MODE_HEAT:
        set_value(0xB0, 0x43);
        set_value(0x80, 0x30);
        break;
      case climate::CLIMATE_MODE_DRY:
        set_value(0xB0, 0x44);
        set_value(0x80, 0x30);
        break;
      case climate::CLIMATE_MODE_HEAT_COOL:
        set_value(0xB0, 0x41);
        set_value(0x80, 0x30);
        break;
      case climate::CLIMATE_MODE_FAN_ONLY:
        set_value(0xB0, 0x45);
        set_value(0x80, 0x30);
        break;
      case climate::CLIMATE_MODE_OFF:
        set_value(0x80, 0x31);
        break;
      default:
        ESP_LOGV(TAG, "Unsupported mode requested");
        break;
    }

    this->mode =
        *call.get_mode();   // Set mode manually since we won't receive a report from the AC if its the same mode again
    this->publish_state();  // Send this state, will get updated once next poll is executed
  }

  if (call.get_target_temperature().has_value()) {
    const uint8_t raw_temperature =
        (*call.get_target_temperature() - this->current_temperature_offset_) * 2;

    ESP_LOGV(TAG, "Requested target temp change to %.2f, %.2f including offset (raw 0x%02X)",
             *call.get_target_temperature(),
             *call.get_target_temperature() - this->current_temperature_offset_, raw_temperature);

    // A set packet that carries the temperature key (0x31) on its own is ignored by the AC, and
    // so is one where the accompanying keys use the default 0x00 flag byte. The original adapter
    // always sends the current mode key (0xB0) with flag 0x02 directly before the temperature
    // key, so mirror that layout here instead of sending 0x31 alone.
    if (this->mode != climate::CLIMATE_MODE_OFF) {
      set_value(0xB0, determine_mode_value(this->mode), 0x02);
    }
    set_value(0x31, raw_temperature, 0x02);

    // The AC silently drops a set packet whose final pair is the temperature key: the last pair's
    // flag byte is overwritten by the checksum, and 0x31 is the one key this unit validates there.
    // Measured on hardware: temperature alone (last pair) is ignored with no ack at all, while the
    // same value is acked and applied as soon as any pair follows it. Repeat the current fan speed
    // as a no-op trailer so the temperature pair is never last. Skipped when this call already
    // changes the fan speed, because that queues 0xA0 itself.
    if (!call.has_custom_fan_mode()) {
      set_value(0xA0, this->last_fan_speed_, 0x00);
    }
  }

  if (call.has_custom_fan_mode()) {
    ESP_LOGV(TAG, "Requested fan mode change");

    const StringRef fanMode = call.get_custom_fan_mode();

    // The genuine adapter's fan command is a single 0xA0 pair (00 A0 01 3x); it
    // does NOT touch 0xB2. Writing 0xB2=0x41 here used to reset a Powerful/Quiet
    // preset back to Normal on every fan-speed change (measured).
    if (fanMode == "Automatic") {
      set_value(0xA0, 0x41);
    } else if (fanMode == "Speed 1") {
      set_value(0xA0, 0x32);
    } else if (fanMode == "Speed 2") {
      set_value(0xA0, 0x33);
    } else if (fanMode == "Speed 3") {
      set_value(0xA0, 0x34);
    } else if (fanMode == "Speed 4") {
      set_value(0xA0, 0x35);
    } else if (fanMode == "Speed 5") {
      set_value(0xA0, 0x36);
    } else
      ESP_LOGV(TAG, "Unsupported fan mode requested");
  }

  if (call.get_swing_mode().has_value()) {
    ESP_LOGV(TAG, "Requested swing mode change");

    switch (*call.get_swing_mode()) {
      case climate::CLIMATE_SWING_BOTH:
        set_value(0xA1, 0x41);
        break;
      case climate::CLIMATE_SWING_OFF:
        // Measured on CS-Z42XKRW: a lone 0xA1 write is silently ignored. The AC
        // only acts on the swing byte when a louver-position pair travels with it,
        // and 0x42 (off) additionally requires 0x35=0x42 in the SAME packet (this
        // is exactly what the genuine adapter sends: 00 A1 01 42 | 00 A5 01 xx |
        // 02 35 01 42). Consequence: choosing a swing mode re-centres the louver.
        set_value(0xA1, 0x42);
        set_value(0xA4, 0x43);
        set_value(0xA5, 0x43);
        set_value(0x35, 0x42, 0x02);
        break;
      case climate::CLIMATE_SWING_VERTICAL:
        set_value(0xA1, 0x43);
        set_value(0xA5, 0x43);
        break;
      case climate::CLIMATE_SWING_HORIZONTAL:
        set_value(0xA1, 0x44);
        set_value(0xA4, 0x43);
        break;
      default:
        ESP_LOGV(TAG, "Unsupported swing mode requested");
        break;
    }
  }

  if (call.has_custom_preset()) {
    ESP_LOGV(TAG, "Requested preset change");

    const StringRef preset = call.get_custom_preset();

    if (preset == "Normal") {
      set_value(0xB2, 0x41);
    } else if (preset == "Powerful") {
      set_value(0xB2, 0x42);
    } else if (preset == "Quiet") {
      set_value(0xB2, 0x43);
    } else
      ESP_LOGV(TAG, "Unsupported preset requested");
  }

  if (this->set_queue_index_ > 0)  // Only send packet if any changes need to be made
  {
    send_set_command();
  }
}

/*
 * Loop handling
 */

void PanasonicACWLAN::handle_poll() {
  if (this->state_ == ACState::Ready && millis() - this->last_packet_sent_ > POLL_INTERVAL) {
    ESP_LOGV(TAG, "Polling AC");
    send_command(CMD_POLL, sizeof(CMD_POLL));
  }
}

void PanasonicACWLAN::handle_init_packets() {
  if (this->state_ == ACState::Initializing) {
    if (millis() - this->init_time_ > INIT_TIMEOUT)  // Handle handshake initialization
    {
      ESP_LOGD(TAG, "Starting handshake [1/16]");
      send_command(CMD_HANDSHAKE_1,
                   sizeof(CMD_HANDSHAKE_1));  // Send first handshake packet, AC won't send a response
      delay(3);                               // Add small delay to mimic real wifi adapter
      send_command(CMD_HANDSHAKE_2,
                   sizeof(CMD_HANDSHAKE_2));  // Send second handshake packet, AC won't send a response
                                              // but we will trigger a resend

      this->state_ = ACState::Handshake;  // Update state to handshake started
    }
  } else if (this->state_ == ACState::FirstPoll &&
             millis() - this->last_packet_sent_ > FIRST_POLL_TIMEOUT)  // Handle sending first poll
  {
    ESP_LOGD(TAG, "Polling for the first time");
    send_command(CMD_POLL, sizeof(CMD_POLL));

    this->state_ = ACState::HandshakeEnding;
  } else if (this->state_ == ACState::HandshakeEnding &&
             millis() - this->last_packet_sent_ > INIT_END_TIMEOUT)  // Handle last handshake message
  {
    ESP_LOGD(TAG, "Finishing handshake [16/16]");
    send_command(CMD_HANDSHAKE_16, sizeof(CMD_HANDSHAKE_16));

    // State is set to ready in the response to this packet
  }
}

bool PanasonicACWLAN::verify_packet() {
  if (this->rx_buffer_.size() < 5)  // Drop packets that are too short
  {
    ESP_LOGW(TAG, "Dropping invalid packet (length)");
    this->rx_buffer_.clear();  // Reset buffer
    return false;
  }

  if (this->rx_buffer_[0] == 0x66)  // Sync packets are the only packet not starting with 0x5A
  {
    ESP_LOGI(TAG, "Received sync packet, triggering initialization");
    this->init_time_ -= INIT_TIMEOUT;  // Set init time back to trigger a initialization now
    this->rx_buffer_.clear();          // Reset buffer
    return false;
  }

  if (this->rx_buffer_[0] != HEADER)  // Check if header matches
  {
    ESP_LOGW(TAG, "Dropping invalid packet (header)");
    this->rx_buffer_.clear();  // Reset buffer
    return false;
  }

  if (this->state_ == ACState::Ready && this->waiting_for_response_)  // If we were waiting for a response, check if the
                                                                      // tx packet counter matches (if we are ready)
  {
    if (this->rx_buffer_[1] != this->transmit_packet_count_ - 1 &&
        this->rx_buffer_[1] != 0xFE)  // Check transmit packet counter
    {
      ESP_LOGW(TAG, "Correcting shifted tx counter");
      this->receive_packet_count_ = this->rx_buffer_[1];
    }
  } else if (this->state_ == ACState::Ready)  // If we were not waiting for a response, check if the rx packet counter
                                              // matches (if we are ready)
  {
    if (this->rx_buffer_[1] != this->receive_packet_count_)  // Check receive packet counter
    {
      ESP_LOGW(TAG, "Correcting shifted rx counter");
      this->receive_packet_count_ = this->rx_buffer_[1];
    }
  }

  uint8_t checksum = 0;  // Set checksum to first byte

  for (uint8_t i : this->rx_buffer_)  // Add all bytes together
    checksum += i;

  if (checksum != 0)  // Check if checksum is valid
  {
    ESP_LOGD(TAG, "Dropping invalid packet (checksum)");

    this->rx_buffer_.clear();  // Reset buffer
    return false;
  }

  return true;
}

/*
 * Field handling
 */

static uint8_t determine_mode_value(climate::ClimateMode mode) {
  switch (mode) {
    case climate::CLIMATE_MODE_HEAT_COOL:
      return 0x41;
    case climate::CLIMATE_MODE_COOL:
      return 0x42;
    case climate::CLIMATE_MODE_HEAT:
      return 0x43;
    case climate::CLIMATE_MODE_DRY:
      return 0x44;
    case climate::CLIMATE_MODE_FAN_ONLY:
      return 0x45;
    default:
      return 0x00;
  }
}

static climate::ClimateMode determine_mode(uint8_t mode) {
  switch (mode)  // Check mode
  {
    case 0x41:  // Auto
      return climate::CLIMATE_MODE_HEAT_COOL;
    case 0x42:  // Cool
      return climate::CLIMATE_MODE_COOL;
    case 0x43:  // Heat
      return climate::CLIMATE_MODE_HEAT;
    case 0x44:  // Dry
      return climate::CLIMATE_MODE_DRY;
    case 0x45:  // Fan only?
      return climate::CLIMATE_MODE_FAN_ONLY;
    default:
      ESP_LOGW(TAG, "Received unknown climate mode");
      return climate::CLIMATE_MODE_OFF;
  }
}

static const char *determine_fan_speed(uint8_t speed) {
  switch (speed) {
    case 0x32:  // 1
      return "Speed 1";
    case 0x33:  // 2
      return "Speed 2";
    case 0x34:  // 3
      return "Speed 3";
    case 0x35:  // 4
      return "Speed 4";
    case 0x36:  // 5
      return "Speed 5";
    case 0x41:  // Auto
      return "Automatic";
    default:
      ESP_LOGW(TAG, "Received unknown fan speed");
      return "Unknown";
  }
}

static const char *determine_preset(uint8_t preset) {
  switch (preset) {
    case 0x43:  // Quiet
      return "Quiet";
    case 0x42:  // Powerful
      return "Powerful";
    case 0x41:  // Normal
      return "Normal";
    default:
      ESP_LOGW(TAG, "Received unknown preset");
      return "Normal";
  }
}

static const char *determine_swing_vertical(uint8_t swing) {
  switch (swing) {
    case 0x42:  // Down
      return "down";
    case 0x45:  // Down center
      return "down_center";
    case 0x43:  // Center
      return "center";
    case 0x44:  // Up Center
      return "up_center";
    case 0x41:  // Up
      return "up";
    default:
      ESP_LOGW(TAG, "Received unknown vertical swing position");
      return "Unknown";
  }
}

static const char *determine_swing_horizontal(uint8_t swing) {
  switch (swing) {
    case 0x42:  // Left
      return "left";
    case 0x5C:  // Left center
      return "left_center";
    case 0x43:  // Center
      return "center";
    case 0x56:  // Right center
      return "right_center";
    case 0x41:  // Right
      return "right";
    default:
      ESP_LOGW(TAG, "Received unknown horizontal swing position");
      return "Unknown";
  }
}

static climate::ClimateSwingMode determine_swing(uint8_t swing) {
  switch (swing) {
    case 0x41:  // Both
      return climate::CLIMATE_SWING_BOTH;
    case 0x42:  // Off
      return climate::CLIMATE_SWING_OFF;
    case 0x43:  // Vertical
      return climate::CLIMATE_SWING_VERTICAL;
    case 0x44:  // Horizontal
      return climate::CLIMATE_SWING_HORIZONTAL;
    default:
      ESP_LOGW(TAG, "Received unknown swing mode");
      return climate::CLIMATE_SWING_OFF;
  }
}

static constexpr bool determine_nanoex(uint8_t nanoex) {
  switch (nanoex) {
    case 0x42:
      return false;
    default:
      return true;
  }
}

/*
 * Packet handling
 */

void PanasonicACWLAN::handle_packet() {
  if (this->rx_buffer_[2] == 0x01 && this->rx_buffer_[3] == 0x01)  // Ping
  {
    ESP_LOGD(TAG, "Answering ping");
    send_command(CMD_PING, sizeof(CMD_PING), CommandType::Response);
  } else if (this->rx_buffer_[2] == 0x10 && this->rx_buffer_[3] == 0x89)  // Received query response
  {
    ESP_LOGD(TAG, "Received query response");

    if (this->rx_buffer_.size() != 125) {
      ESP_LOGW(TAG, "Received invalid query response");
      return;
    }

    if (this->rx_buffer_[14] == 0x31)          // Check if power state is off
      this->mode = climate::CLIMATE_MODE_OFF;  // Climate is off
    else {
      this->mode = determine_mode(this->rx_buffer_[18]);  // Check mode if power state is not off
    }

    update_target_temperature((int8_t) this->rx_buffer_[22]);
    update_current_temperature((int8_t) this->rx_buffer_[62]);
    update_outside_temperature((int8_t) this->rx_buffer_[66]);  // Set current (outside) temperature

    StringRef horizontalSwing(determine_swing_horizontal(this->rx_buffer_[34]));
    StringRef verticalSwing(determine_swing_vertical(this->rx_buffer_[38]));

    update_swing_horizontal(horizontalSwing);
    update_swing_vertical(verticalSwing);

    bool nanoex = determine_nanoex(this->rx_buffer_[50]);

    update_nanoex(nanoex);

    // Key 0x34 discriminates iAUTO-X (0x43) from plain auto (0x42); its value
    // sits at offset 54 in the 125-byte query response (pairs start at 11,
    // stride 4, value at +2).
    update_iautox(this->rx_buffer_[54] == 0x43);

    this->set_custom_fan_mode_(determine_fan_speed(this->rx_buffer_[26]));
    this->set_custom_preset_(determine_preset(this->rx_buffer_[42]));

    this->swing_mode = this->apply_swing_mask(determine_swing(this->rx_buffer_[30]));

    // climate::ClimateAction action = determine_action(); // Determine the current action of the AC
    // this->action = action;

    this->publish_state();
  } else if (this->rx_buffer_[2] == 0x10 && this->rx_buffer_[3] == 0x88)  // Command ack
  {
    ESP_LOGV(TAG, "Received command ack");
  } else if (this->rx_buffer_[2] == 0x10 && this->rx_buffer_[3] == 0x0A)  // Report
  {
    ESP_LOGV(TAG, "Received report");
    send_command(CMD_REPORT_ACK, sizeof(CMD_REPORT_ACK), CommandType::Response);

    if (this->rx_buffer_.size() < 13) {
      ESP_LOGE(TAG, "Report is too short to handle");
      return;
    }

    // 0 = Header & packet type
    // 1 = Packet length
    // 2 = Key value pair counter
    for (int i = 0; i < this->rx_buffer_[10]; i++) {
      // Offset everything by header, packet length and pair counter (4 * 3)
      // then offset by pair length (i * 4)
      int currentIndex = (4 * 3) + (i * 4);

      // 0 = Header
      // 1 = Data
      // 2 = Data
      // 3 = ?
      switch (this->rx_buffer_[currentIndex]) {
        case 0x80:  // Power mode
          switch (this->rx_buffer_[currentIndex + 2]) {
            case 0x30:  // Power mode on
              ESP_LOGV(TAG, "Received power mode on");
              // Ignore power on and let mode be set by other report
              break;
            case 0x31:  // Power mode off
              ESP_LOGV(TAG, "Received power mode off");
              this->mode = climate::CLIMATE_MODE_OFF;
              break;
            default:
              ESP_LOGW(TAG, "Received unknown power mode");
              break;
          }
          break;
        case 0xB0:  // Mode
          this->mode = determine_mode(this->rx_buffer_[currentIndex + 2]);
          break;
        case 0x31:  // Target temperature
          ESP_LOGV(TAG, "Received target temperature");
          update_target_temperature((int8_t) this->rx_buffer_[currentIndex + 2]);
          break;
        case 0xA0:  // Fan speed
          ESP_LOGV(TAG, "Received fan speed");
          this->last_fan_speed_ = this->rx_buffer_[currentIndex + 2];
          this->set_custom_fan_mode_(determine_fan_speed(this->last_fan_speed_));
          break;
        case 0xB2:  // Preset
          ESP_LOGV(TAG, "Received preset");
          this->set_custom_preset_(determine_preset(this->rx_buffer_[currentIndex + 2]));
          break;
        case 0xA1:
          ESP_LOGV(TAG, "Received swing mode");
          this->swing_mode = this->apply_swing_mask(determine_swing(this->rx_buffer_[currentIndex + 2]));
          break;
        case 0xA5:  // Horizontal swing position
          ESP_LOGV(TAG, "Received horizontal swing position");

          update_swing_horizontal(StringRef(determine_swing_horizontal(this->rx_buffer_[currentIndex + 2])));
          break;
        case 0xA4:  // Vertical swing position
          ESP_LOGV(TAG, "Received vertical swing position");

          update_swing_vertical(StringRef(determine_swing_vertical(this->rx_buffer_[currentIndex + 2])));
          break;
        case 0x33:  // nanoex mode
          ESP_LOGV(TAG, "Received nanoex state");

          update_nanoex(determine_nanoex(this->rx_buffer_[currentIndex + 2]));
          break;
        case 0x34:  // iAUTO-X flag (0x43 = iAUTO-X, 0x42 = plain auto)
          ESP_LOGV(TAG, "Received iAUTO-X state");
          update_iautox(this->rx_buffer_[currentIndex + 2] == 0x43);
          break;
        case 0x20:
          ESP_LOGV(TAG, "Received unknown nanoex field");
          // Not sure what this one, ignore it for now
          break;
        default:
          ESP_LOGW(TAG, "Report has unknown field");
          break;
      }
    }

    climate::ClimateAction action = determine_action();  // Determine the current action of the AC
    this->action = action;

    this->publish_state();
  } else if (this->rx_buffer_[2] == 0x01 && this->rx_buffer_[3] == 0x80)  // Answer for handshake 16
  {
    ESP_LOGI(TAG, "Panasonic AC component v%s initialized", VERSION);
    this->state_ = ACState::Ready;
  } else {
    ESP_LOGW(TAG, "Received unknown packet");
  }
}

void PanasonicACWLAN::handle_handshake_packet() {
  if (this->rx_buffer_[2] == 0x00 && this->rx_buffer_[3] == 0x89)  // Answer for handshake 2
  {
    ESP_LOGD(TAG, "Answering handshake [2/16]");
    send_command(CMD_HANDSHAKE_3, sizeof(CMD_HANDSHAKE_3));
  } else if (this->rx_buffer_[2] == 0x00 && this->rx_buffer_[3] == 0x8C)  // Answer for handshake 3
  {
    ESP_LOGD(TAG, "Answering handshake [3/16]");
    send_command(CMD_HANDSHAKE_4, sizeof(CMD_HANDSHAKE_4));
  } else if (this->rx_buffer_[2] == 0x00 && this->rx_buffer_[3] == 0x90)  // Answer for handshake 4
  {
    ESP_LOGD(TAG, "Answering handshake [4/16]");
    send_command(CMD_HANDSHAKE_5, sizeof(CMD_HANDSHAKE_5));
  } else if (this->rx_buffer_[2] == 0x00 && this->rx_buffer_[3] == 0x91)  // Answer for handshake 5
  {
    ESP_LOGD(TAG, "Answering handshake [5/16]");
    send_command(CMD_HANDSHAKE_6, sizeof(CMD_HANDSHAKE_6));
  } else if (this->rx_buffer_[2] == 0x00 && this->rx_buffer_[3] == 0x92)  // Answer for handshake 6
  {
    ESP_LOGD(TAG, "Answering handshake [6/16]");
    send_command(CMD_HANDSHAKE_7, sizeof(CMD_HANDSHAKE_7));
  } else if (this->rx_buffer_[2] == 0x00 && this->rx_buffer_[3] == 0xC1)  // Answer for handshake 7
  {
    ESP_LOGD(TAG, "Answering handshake [7/16]");
    send_command(CMD_HANDSHAKE_8, sizeof(CMD_HANDSHAKE_8));
  } else if (this->rx_buffer_[2] == 0x01 && this->rx_buffer_[3] == 0xCC)  // Answer for handshake 8
  {
    ESP_LOGD(TAG, "Answering handshake [8/16]");
    send_command(CMD_HANDSHAKE_9, sizeof(CMD_HANDSHAKE_9));
  } else if (this->rx_buffer_[2] == 0x10 && this->rx_buffer_[3] == 0x80)  // Answer for handshake 9
  {
    ESP_LOGD(TAG, "Answering handshake [9/16]");
    send_command(CMD_HANDSHAKE_10, sizeof(CMD_HANDSHAKE_10));
  } else if (this->rx_buffer_[2] == 0x10 && this->rx_buffer_[3] == 0x81)  // Answer for handshake 10
  {
    ESP_LOGD(TAG, "Answering handshake [10/16]");
    send_command(CMD_HANDSHAKE_11, sizeof(CMD_HANDSHAKE_11));
  } else if (this->rx_buffer_[2] == 0x00 && this->rx_buffer_[3] == 0x98)  // Answer for handshake 11
  {
    ESP_LOGD(TAG, "Answering handshake [11/16]");
    send_command(CMD_HANDSHAKE_12, sizeof(CMD_HANDSHAKE_12));
  } else if (this->rx_buffer_[2] == 0x01 && this->rx_buffer_[3] == 0x80)  // Answer for handshake 12
  {
    ESP_LOGD(TAG, "Answering handshake [12/16]");
    send_command(CMD_HANDSHAKE_13, sizeof(CMD_HANDSHAKE_13));
  } else if (this->rx_buffer_[2] == 0x10 && this->rx_buffer_[3] == 0x88)  // Answer for handshake 13
  {
    // Ignore
    ESP_LOGD(TAG, "Ignoring handshake [13/16]");
  } else if (this->rx_buffer_[2] == 0x01 &&
             this->rx_buffer_[3] == 0x09)  // First unsolicited packet from AC containing rx counter
  {
    ESP_LOGD(TAG, "Received rx counter [14/16]");
    this->receive_packet_count_ = this->rx_buffer_[1];  // Set rx packet counter
    send_command(CMD_HANDSHAKE_14, sizeof(CMD_HANDSHAKE_14), CommandType::Response);
  } else if (this->rx_buffer_[2] == 0x00 && this->rx_buffer_[3] == 0x20)  // Second unsolicited packet from AC
  {
    ESP_LOGD(TAG, "Answering handshake [15/16]");
    this->state_ = ACState::FirstPoll;  // Start delayed first poll
    send_command(CMD_HANDSHAKE_15, sizeof(CMD_HANDSHAKE_15), CommandType::Response);
  } else {
    ESP_LOGW(TAG, "Received unknown packet during initialization");
  }
}

/*
 * Packet sending
 */

void PanasonicACWLAN::send_set_command() {
  if (this->waiting_for_response_) {
    // A set packet is still waiting for its ack; queue these pairs and let loop()
    // send them as soon as the response lands. Keeps the tx/rx counters in sync.
    this->pending_set_send_ = true;
    return;
  }

  // Size of packet is 3 * 4 (for the header, packet size, and key value pair counter)
  // setQueueIndex * 4 for the individual key value pairs
  int packetLength = (3 * 4) + (this->set_queue_index_ * 4);
  std::vector<uint8_t> packet(packetLength);

  // Mark this packet as a set command
  packet[2] = 0x10;
  packet[3] = 0x08;

  packet[4] = 0x00;                                  // Packet size header
  packet[5] = (4 * this->set_queue_index_) - 1 + 6;  // Packet size, key value pairs * 4, subtract checksum (-1), add 4
                                                     // for key value pair counter and 2 for rest of packet size
  packet[6] = 0x01;
  packet[7] = 0x01;

  packet[8] = 0x30;  // Key value pair counter header
  packet[9] = 0x01;
  packet[10] = this->set_queue_index_;  // Key value pair counter

  // Wire layout is [flag][key][0x01][value] per pair, starting at byte 11, so the
  // flag byte belongs to the pair that follows it. The original adapter's captures
  // show flags of 0x00 and 0x02 on individual pairs (e.g. nanoeX off is
  // 02 33 01 42, temperature is 02 31 01 xx). Writing the flag after the value
  // instead silently shifted it into the next pair's slot, which made it
  // impossible to set a flag on the last (or only) pair — a single-pair write
  // always went out as flag 0x00, which the AC accepts and then ignores.
  for (int i = 0; i < this->set_queue_index_; i++) {
    packet[11 + (i * 4) + 0] = this->set_queue_[i][2];  // Flag byte (0x00 or 0x02)
    packet[11 + (i * 4) + 1] = this->set_queue_[i][0];  // Key
    packet[11 + (i * 4) + 2] = 0x01;                    // Unknown, always 0x01
    packet[11 + (i * 4) + 3] = this->set_queue_[i][1];  // Value
  }

  send_packet(packet, CommandType::Normal);
  this->set_queue_index_ = 0;
}

void PanasonicACWLAN::send_command(const uint8_t *command, size_t commandLength, CommandType type) {
  std::vector<uint8_t> packet(commandLength + 3);  // Reserve space for upcoming packet

  for (int i = 0; i < commandLength; i++)  // Loop through command
  {
    packet[i + 2] = command[i];  // Add to packet
  }

  this->last_command_ = command;               // Store the last command we sent
  this->last_command_length_ = commandLength;  // Store the length of the last command we sent

  send_packet(packet, type);  // Actually send the constructed packet
}

void PanasonicACWLAN::send_packet(std::vector<uint8_t> packet, CommandType type) {
  uint8_t length = packet.size();

  uint8_t checksum = 0;  // Checksum is calculated by adding all bytes together
  packet[0] = HEADER;    // Write header to packet

  uint8_t packetCount = this->transmit_packet_count_;  // Set packet counter

  if (type == CommandType::Response)
    packetCount = this->receive_packet_count_;  // Set the packet counter to the rx counter
  else if (type == CommandType::Resend)
    packetCount = this->transmit_packet_count_ -
                  1;  // Set the packet counter to the tx counter -1 (we are sending the same packet again)

  packet[1] = packetCount;  // Write to packet

  for (uint8_t i : packet)  // Loop through payload to calculate checksum
  {
    checksum += i;  // Add byte to checksum
  }

  checksum = (~checksum + 1);     // Compute checksum
  packet[length - 1] = checksum;  // Add checksum to end of packet

  this->last_packet_sent_ = millis();  // Save the time when we sent the last packet

  if (type == CommandType::Normal)  // Do not increase tx counter if this was a response or if this was a resent packet
  {
    if (this->transmit_packet_count_ == 0xFE)
      this->transmit_packet_count_ = 0x01;  // Special case, roll over transmit counter after 0xFE
    else
      this->transmit_packet_count_++;  // Increase tx packet counter if this wasn't a response
  } else if (type == CommandType::Response) {
    if (this->receive_packet_count_ == 0xFE)
      this->receive_packet_count_ = 0x01;  // Special case, roll over receive counter after 0xFE
    else
      this->receive_packet_count_++;  // Increase rx counter if this was a response
  }

  if (type != CommandType::Response)     // Don't wait for a response for responses
    this->waiting_for_response_ = true;  // Mark that we are waiting for a response

  write_array(packet);       // Write to UART
  log_packet(packet, true);  // Write to log
}

/*
 * Helpers
 */
void PanasonicACWLAN::handle_resend() {
  if (this->waiting_for_response_ && millis() - this->last_packet_sent_ > RESPONSE_TIMEOUT &&
      this->rx_buffer_.empty())  // Check if AC failed to respond in time and resend packet, if nothing was received yet
  {
    ESP_LOGD(TAG, "Resending previous packet");
    send_command(this->last_command_, this->last_command_length_, CommandType::Resend);
  }
}

void PanasonicACWLAN::set_value(uint8_t key, uint8_t value, uint8_t flag) {
  if (this->set_queue_index_ >= 15) {
    ESP_LOGE(TAG, "Set queue overflow");
    this->set_queue_index_ = 0;
    return;
  }

  this->set_queue_[this->set_queue_index_][0] = key;
  this->set_queue_[this->set_queue_index_][1] = value;
  this->set_queue_[this->set_queue_index_][2] = flag;
  this->set_queue_index_++;
}

/*
 * Sensor handling
 */

void PanasonicACWLAN::on_vertical_swing_change(const StringRef& swing) {
  if (this->state_ != ACState::Ready)
    return;

  ESP_LOGD(TAG, "Setting vertical swing position");

  if (swing == "down")
    set_value(0xA4, 0x42);
  else if (swing == "down_center")
    set_value(0xA4, 0x45);
  else if (swing == "center")
    set_value(0xA4, 0x43);
  else if (swing == "up_center")
    set_value(0xA4, 0x44);
  else if (swing == "up")
    set_value(0xA4, 0x41);
  else if (swing == "swing") {
    // Continuous up/down sweep. Measured on CS-Z42XKRW: A1=0x43 ("vertical")
    // is acked and echoed but the flap does not move; only A1=0x41 ("both")
    // actually sweeps this unit's flap.
    set_value(0xA1, 0x41);
  } else if (swing == "auto") {
    set_value(0xA1, 0x41);  // both axes sweeping (only the flap moves here)
  } else {
    ESP_LOGW(TAG, "Unsupported vertical swing position received");
    return;
  }

  send_set_command();
}

void PanasonicACWLAN::on_horizontal_swing_change(const StringRef &swing) {
  if (this->state_ != ACState::Ready)
    return;

  ESP_LOGD(TAG, "Setting horizontal swing position");

  if (swing == "left")
    set_value(0xA5, 0x42);
  else if (swing == "left_center")
    set_value(0xA5, 0x5C);
  else if (swing == "center")
    set_value(0xA5, 0x43);
  else if (swing == "right_center")
    set_value(0xA5, 0x56);
  else if (swing == "right")
    set_value(0xA5, 0x41);

  send_set_command();
}

void PanasonicACWLAN::on_nanoex_change(bool state) {
  if (this->state_ != ACState::Ready)
    return;

  if (state) {
    ESP_LOGV(TAG, "Turning nanoex on");
    // Genuine adapter capture (protocol/logic_analyzer/controller/nanoe_off_on.dsl):
    //   on  -> 10.08 01 01 30 01 01 02 33 01 45
    //   off -> 10.08 01 01 30 01 01 02 33 01 42
    // Single pair, flag byte 0x02. The AC reports 0x43 for on and 0x42 for off.
    set_value(0x33, 0x45, 0x02);  // nanoeX on
  } else {
    ESP_LOGV(TAG, "Turning nanoex off");
    set_value(0x33, 0x42, 0x02);  // nanoeX off
  }

  send_set_command();
}

void PanasonicACWLAN::raw_set(uint8_t key, uint8_t value, uint8_t flag) {
  if (this->state_ != ACState::Ready)
    return;

  ESP_LOGW(TAG, "RAW set 0x%02X = 0x%02X (flag 0x%02X)", key, value, flag);
  set_value(key, value, flag);
  send_set_command();
}

void PanasonicACWLAN::raw_set2(uint8_t key1, uint8_t value1, uint8_t key2, uint8_t value2, uint8_t flag) {
  if (this->state_ != ACState::Ready)
    return;

  ESP_LOGW(TAG, "RAW set 0x%02X = 0x%02X, 0x%02X = 0x%02X (flag 0x%02X)", key1, value1, key2, value2, flag);
  set_value(key1, value1, flag);
  set_value(key2, value2, flag);
  send_set_command();
}

void PanasonicACWLAN::on_iautox_change(bool state) {
  if (this->state_ != ACState::Ready)
    return;

  // iAUTO-X and plain AUTO share mode byte 0xB0 = 0x41; the discriminator is
  // key 0x34 (0x43 = iAUTO-X, 0x42 = plain auto). The genuine adapter sends
  // 0x34 with flag 0x02 (see quiet_on_off_powerful_on_off.dsl: 02 34 01 42).
  if (state) {
    ESP_LOGV(TAG, "Turning iAUTO-X on");
    set_value(0xB0, 0x41);        // mode auto
    set_value(0x34, 0x43, 0x02);  // iAUTO-X flag
    set_value(0x80, 0x30);        // power on
  } else {
    ESP_LOGV(TAG, "Turning iAUTO-X off");
    set_value(0x34, 0x42, 0x02);  // back to plain auto
  }

  send_set_command();
}

void PanasonicACWLAN::on_eco_change(bool state) {
  if (this->state_ != ACState::Ready)
    return;

  return;

  // TODO: implement eco

  // if (state) {
  //   ESP_LOGV(TAG, "Turning eco on");
  //   set_value(..., ...);  // eco on
  // } else {
  //   ESP_LOGV(TAG, "Turning eco off");
  //   set_value(..., ...);  // eco off
  // }

  // send_set_command();
}

void PanasonicACWLAN::on_econavi_change(bool state) {
  if (this->state_ != ACState::Ready)
    return;

  return;

  // TODO: implement econavi

  // if (state) {
  //   ESP_LOGV(TAG, "Turning econavi on");
  //   set_value(..., ...);  // econavi on
  // } else {
  //   ESP_LOGV(TAG, "Turning econavi off");
  //   set_value(..., ...);  // econavi off
  // }

  // send_set_command();
}

void PanasonicACWLAN::on_mild_dry_change(bool state) {
  if (this->state_ != ACState::Ready)
    return;

  return;

  // TODO: implement mild_dry

  // if (state) {
  //   ESP_LOGV(TAG, "Turning mild_dry on");
  //   set_value(..., ...);  // mild_dry on
  // } else {
  //   ESP_LOGV(TAG, "Turning mild_dry off");
  //   set_value(..., ...);  // mild_dry off
  // }

  // send_set_command();
}

}  // namespace WLAN
}  // namespace panasonic_ac
}  // namespace esphome
