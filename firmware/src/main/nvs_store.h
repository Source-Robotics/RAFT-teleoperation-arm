// Settings storage on the ESP32: the PersistentConfig blob in NVS (Preferences namespace "raft", key "cfg").
// The blob carries its own magic, version and CRC (config_store.h), so a half-written or old blob is
// rejected and the App falls back to defaults. ESP32 / Arduino only.
#pragma once

#include "config_store.h"

namespace raft {

class NvsStorage : public Storage {
public:
    bool load(PersistentConfig& c) override;
    bool save(const PersistentConfig& c) override;
};

}  // namespace raft
