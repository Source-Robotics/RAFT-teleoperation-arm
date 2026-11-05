// Tiny reader for the flat JSON command lines of the RAFT command channel ({"cmd": "...", "key": value}).
// Not a general JSON parser: top-level keys, string / number / bool / array-of-numbers values. Portable.
#pragma once

#include <stddef.h>

namespace raft {
namespace json {

bool getString(const char* line, const char* key, char* out, size_t cap);   // escapes and control chars -> '?'
// Numbers must be finite and at most 1e9 in magnitude: "inf", "nan" and "1e999" count as not a number.
bool getNumber(const char* line, const char* key, double& out);
bool getBool(const char* line, const char* key, bool& out);
int  getNumberArray(const char* line, const char* key, double* out, int maxN);   // count read, -1 if missing or bad

}  // namespace json
}  // namespace raft
