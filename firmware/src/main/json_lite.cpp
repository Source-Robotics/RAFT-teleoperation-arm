#include "json_lite.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

namespace raft {
namespace json {
namespace {

// No command takes a number this large; strtod also reads "inf", "nan" and "1e999" (= inf), which are refused.
const double kMaxMagnitude = 1e9;

// strtod plus the checks above; end = first character after the number
bool number(const char* v, const char** end, double& out)
{
    char* e = nullptr;
    const double d = strtod(v, &e);
    if (e == v || !isfinite(d) || fabs(d) > kMaxMagnitude) return false;
    out = d;
    *end = e;
    return true;
}

// pointer to the first character of key's value, or null
const char* valueOf(const char* s, const char* key)
{
    char pat[48];
    const int n = snprintf(pat, sizeof(pat), "\"%s\"", key);
    if (n <= 0 || n >= (int)sizeof(pat)) return nullptr;
    for (const char* p = strstr(s, pat); p; p = strstr(p + 1, pat)) {
        const char* q = p + n;
        while (*q == ' ' || *q == '\t') ++q;
        if (*q != ':') continue;
        ++q;
        while (*q == ' ' || *q == '\t') ++q;
        return q;
    }
    return nullptr;
}

}  // namespace

bool getString(const char* s, const char* key, char* out, size_t cap)
{
    const char* v = valueOf(s, key);
    if (!v || *v != '"' || cap == 0) return false;
    ++v;
    size_t n = 0;
    while (*v && *v != '"') {
        // values are plain words (commands, modes, host names): an escape or a control character comes out
        // as '?', so a value echoed into a reply can never break its JSON
        char c = *v;
        if (c == '\\' && v[1]) { ++v; c = '?'; }
        if ((unsigned char)c < 0x20) c = '?';
        if (n + 1 < cap) out[n++] = c;
        ++v;
    }
    out[n] = '\0';
    return *v == '"';
}

bool getNumber(const char* s, const char* key, double& out)
{
    const char* v = valueOf(s, key);
    if (!v) return false;
    const char* end = nullptr;
    return number(v, &end, out);
}

bool getBool(const char* s, const char* key, bool& out)
{
    const char* v = valueOf(s, key);
    if (!v) return false;
    if (strncmp(v, "true", 4) == 0) { out = true; return true; }
    if (strncmp(v, "false", 5) == 0) { out = false; return true; }
    double d;
    if (getNumber(s, key, d)) { out = d != 0.0; return true; }
    return false;
}

int getNumberArray(const char* s, const char* key, double* out, int maxN)
{
    const char* v = valueOf(s, key);
    if (!v || *v != '[') return -1;
    ++v;
    int n = 0;
    while (*v) {
        while (*v == ' ' || *v == '\t') ++v;
        if (*v == ']') break;
        const char* end = nullptr;
        double d = 0.0;
        if (!number(v, &end, d)) return -1;
        if (n < maxN) out[n] = d;
        ++n;
        v = end;
        while (*v == ' ' || *v == '\t') ++v;
        if (*v == ',') ++v;
    }
    return n > maxN ? maxN : n;
}

}  // namespace json
}  // namespace raft
