#include "nvs_store.h"

#include <Preferences.h>

namespace raft {
namespace {
const char* kNamespace = "raft";
const char* kKey = "cfg";
}  // namespace

bool NvsStorage::load(PersistentConfig& c)
{
    Preferences p;
    if (!p.begin(kNamespace, true)) return false;      // read-only; fails when the namespace does not exist yet
    uint8_t buf[sizeof(PersistentConfig)];
    const size_t n = p.getBytes(kKey, buf, sizeof(buf));   // an older, shorter blob comes back with its own length
    p.end();
    return n > 0 && deserializeConfig(buf, n, c);
}

bool NvsStorage::save(const PersistentConfig& c)
{
    uint8_t buf[sizeof(PersistentConfig)];
    const size_t n = serializeConfig(c, buf, sizeof(buf));
    if (!n) return false;
    Preferences p;
    if (!p.begin(kNamespace, false)) return false;
    const size_t w = p.putBytes(kKey, buf, n);
    p.end();
    return w == n;
}

}  // namespace raft
