// The mission documents' text (MissionJson.h; docs/sdk/mission-training.md, section 6).

#include "sdk/MissionJson.h"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <set>
#include <utility>

namespace fsim::mission {

namespace {

constexpr int kMaxDepth = 256;

class StrictParser {
public:
    StrictParser(std::string_view text, std::string_view what) : s_(text), what_(what) {}

    core::Json document() {
        if (s_.size() >= 3 && static_cast<unsigned char>(s_[0]) == 0xEF && static_cast<unsigned char>(s_[1]) == 0xBB &&
            static_cast<unsigned char>(s_[2]) == 0xBF)
            fail("a byte order mark");
        space();
        if (peek() != '{') fail("a document is a JSON object");
        core::Json value = parseValue(0);
        space();
        if (i_ != s_.size()) fail("text after the document");
        return value;
    }

private:
    [[noreturn]] void fail(const std::string& why) const {
        throw Malformed(std::string(what_) + ": not strict JSON: " + why + " at byte " + std::to_string(i_));
    }
    char peek() const noexcept { return i_ < s_.size() ? s_[i_] : '\0'; }
    bool done() const noexcept { return i_ >= s_.size(); }
    void space() noexcept {
        while (!done() && (s_[i_] == ' ' || s_[i_] == '\t' || s_[i_] == '\n' || s_[i_] == '\r')) ++i_;
    }
    void expect(char c) {
        if (peek() != c) fail(std::string("'") + c + "' expected");
        ++i_;
    }
    void literal(std::string_view word) {
        if (s_.substr(i_, word.size()) != word) fail("not a JSON value");
        i_ += word.size();
    }

    core::Json parseValue(int depth) {
        if (depth > kMaxDepth) fail("nested too deeply");
        space();
        switch (peek()) {
        case '{': return parseObject(depth);
        case '[': return parseArray(depth);
        case '"': return core::Json(parseString());
        case 't': literal("true"); return core::Json(true);
        case 'f': literal("false"); return core::Json(false);
        case 'n': literal("null"); return core::Json(nullptr);
        default: return core::Json(parseNumber());
        }
    }

    core::Json parseObject(int depth) {
        expect('{');
        core::Json::Object members;
        std::set<std::string> names;
        space();
        if (peek() == '}') {
            ++i_;
            return core::Json(std::move(members));
        }
        for (;;) {
            space();
            if (peek() != '"') fail("a member name expected");
            std::string name = parseString();
            if (!names.insert(name).second) fail("duplicate member \"" + name + "\"");
            space();
            expect(':');
            core::Json value = parseValue(depth + 1);
            members.emplace_back(std::move(name), std::move(value));
            space();
            if (peek() == ',') {
                ++i_;
                continue;
            }
            expect('}');
            return core::Json(std::move(members));
        }
    }

    core::Json parseArray(int depth) {
        expect('[');
        core::Json::Array items;
        space();
        if (peek() == ']') {
            ++i_;
            return core::Json(std::move(items));
        }
        for (;;) {
            items.push_back(parseValue(depth + 1));
            space();
            if (peek() == ',') {
                ++i_;
                continue;
            }
            expect(']');
            return core::Json(std::move(items));
        }
    }

    unsigned hex4() {
        if (i_ + 4 > s_.size()) fail("a short \\u escape");
        unsigned v = 0;
        for (int k = 0; k < 4; ++k) {
            const char c = s_[i_++];
            v <<= 4;
            if (c >= '0' && c <= '9') v |= static_cast<unsigned>(c - '0');
            else if (c >= 'a' && c <= 'f') v |= static_cast<unsigned>(c - 'a' + 10);
            else if (c >= 'A' && c <= 'F') v |= static_cast<unsigned>(c - 'A' + 10);
            else fail("a \\u escape that is not hexadecimal");
        }
        return v;
    }
    static void put(std::string& out, unsigned cp) {
        if (cp < 0x80) {
            out += static_cast<char>(cp);
        } else if (cp < 0x800) {
            out += static_cast<char>(0xC0 | (cp >> 6));
            out += static_cast<char>(0x80 | (cp & 0x3F));
        } else if (cp < 0x10000) {
            out += static_cast<char>(0xE0 | (cp >> 12));
            out += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
            out += static_cast<char>(0x80 | (cp & 0x3F));
        } else {
            out += static_cast<char>(0xF0 | (cp >> 18));
            out += static_cast<char>(0x80 | ((cp >> 12) & 0x3F));
            out += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
            out += static_cast<char>(0x80 | (cp & 0x3F));
        }
    }

    /// One UTF-8 sequence, well-formed (no overlong form, no surrogate, nothing above U+10FFFF), copied as it is.
    void utf8(std::string& out) {
        const auto byte = [&](std::size_t k) { return static_cast<unsigned char>(s_[k]); };
        const unsigned char b0 = byte(i_);
        std::size_t n = 0;
        unsigned cp = 0;
        if (b0 >= 0xC2 && b0 <= 0xDF) n = 1, cp = b0 & 0x1Fu;
        else if (b0 >= 0xE0 && b0 <= 0xEF) n = 2, cp = b0 & 0x0Fu;
        else if (b0 >= 0xF0 && b0 <= 0xF4) n = 3, cp = b0 & 0x07u;
        else fail("not UTF-8");
        if (i_ + n >= s_.size()) fail("not UTF-8");
        for (std::size_t k = 1; k <= n; ++k) {
            const unsigned char b = byte(i_ + k);
            if ((b & 0xC0) != 0x80) fail("not UTF-8");
            cp = (cp << 6) | (b & 0x3Fu);
        }
        if ((n == 2 && cp < 0x800) || (n == 3 && cp < 0x10000) || cp > 0x10FFFF || (cp >= 0xD800 && cp <= 0xDFFF)) fail("not UTF-8");
        out.append(s_.substr(i_, n + 1));
        i_ += n + 1;
    }

    std::string parseString() {
        expect('"');
        std::string out;
        for (;;) {
            if (done()) fail("an unterminated string");
            const unsigned char c = static_cast<unsigned char>(s_[i_]);
            if (c == '"') {
                ++i_;
                return out;
            }
            if (c < 0x20) fail("a control character in a string");
            if (c >= 0x80) {
                utf8(out);
                continue;
            }
            ++i_;
            if (c != '\\') {
                out += static_cast<char>(c);
                continue;
            }
            if (done()) fail("an unterminated escape");
            const char e = s_[i_++];
            switch (e) {
            case '"': out += '"'; break;
            case '\\': out += '\\'; break;
            case '/': out += '/'; break;
            case 'b': out += '\b'; break;
            case 'f': out += '\f'; break;
            case 'n': out += '\n'; break;
            case 'r': out += '\r'; break;
            case 't': out += '\t'; break;
            case 'u': {
                unsigned cp = hex4();
                if (cp >= 0xDC00 && cp <= 0xDFFF) fail("a lone surrogate");
                if (cp >= 0xD800 && cp <= 0xDBFF) {
                    if (s_.substr(i_, 2) != "\\u") fail("a lone surrogate");
                    i_ += 2;
                    const unsigned low = hex4();
                    if (low < 0xDC00 || low > 0xDFFF) fail("a lone surrogate");
                    cp = 0x10000 + ((cp - 0xD800) << 10) + (low - 0xDC00);
                }
                put(out, cp);
                break;
            }
            default: fail("an invalid escape");
            }
        }
    }

    double parseNumber() {
        const std::size_t start = i_;
        const auto digits = [&] {
            const std::size_t from = i_;
            while (!done() && s_[i_] >= '0' && s_[i_] <= '9') ++i_;
            return i_ - from;
        };
        if (peek() == '-') ++i_;
        if (peek() == '0') {
            ++i_;
        } else if (peek() >= '1' && peek() <= '9') {
            digits();
        } else {
            fail("not a JSON value");
        }
        if (peek() == '.') {
            ++i_;
            if (digits() == 0) fail("a fraction without digits");
        }
        if (peek() == 'e' || peek() == 'E') {
            ++i_;
            if (peek() == '+' || peek() == '-') ++i_;
            if (digits() == 0) fail("an exponent without digits");
        }
        double v = 0.0;
        const auto [end, ec] = std::from_chars(s_.data() + start, s_.data() + i_, v);
        if (ec == std::errc::result_out_of_range) {
            // from_chars reports underflow the same way: a value too small for a double is 0, as RFC 8259 readers
            // take it; one too large overflows and is refused.
            bool zero = true;
            for (std::size_t k = start; k < i_ && (s_[k] != 'e' && s_[k] != 'E'); ++k)
                if (s_[k] >= '1' && s_[k] <= '9') zero = false;
            const std::size_t e = s_.find_first_of("eE", start);
            const bool negativeExponent = e != std::string_view::npos && e < i_ && e + 1 < i_ && s_[e + 1] == '-';
            if (!zero && !negativeExponent) fail("a number a double cannot hold");
            return 0.0;
        }
        if (ec != std::errc() || end != s_.data() + i_ || !std::isfinite(v)) fail("a number a double cannot hold");
        return v;
    }

    std::string_view s_;
    std::string_view what_;
    std::size_t i_ = 0;
};

void writeNumber(std::string& out, double v) {
    if (!std::isfinite(v)) {
        out += "null";
        return;
    }
    if (v == 0.0) v = 0.0; // (no negative zero)
    char buffer[64];
    const auto [end, ec] = std::to_chars(buffer, buffer + sizeof buffer, v);
    out.append(buffer, ec == std::errc() ? static_cast<std::size_t>(end - buffer) : 0);
}

void writeString(std::string& out, std::string_view s) {
    out += '"';
    for (const char ch : s) {
        const unsigned char c = static_cast<unsigned char>(ch);
        switch (c) {
        case '"': out += "\\\""; break;
        case '\\': out += "\\\\"; break;
        case '\b': out += "\\b"; break;
        case '\f': out += "\\f"; break;
        case '\n': out += "\\n"; break;
        case '\r': out += "\\r"; break;
        case '\t': out += "\\t"; break;
        default:
            if (c < 0x20) {
                static const char* const hex = "0123456789abcdef";
                out += "\\u00";
                out += hex[c >> 4];
                out += hex[c & 0xF];
            } else {
                out += ch;
            }
        }
    }
    out += '"';
}

void canonicalTo(std::string& out, const core::Json& v) {
    switch (v.type()) {
    case core::Json::Type::Null: out += "null"; break;
    case core::Json::Type::Bool: out += v.asBool() ? "true" : "false"; break;
    case core::Json::Type::Number: writeNumber(out, v.asNumber()); break;
    case core::Json::Type::String: writeString(out, v.asString()); break;
    case core::Json::Type::Array: {
        out += '[';
        bool first = true;
        for (const core::Json& item : v.asArray()) {
            if (!first) out += ',';
            first = false;
            canonicalTo(out, item);
        }
        out += ']';
        break;
    }
    case core::Json::Type::Object: {
        std::vector<const std::pair<std::string, core::Json>*> members;
        for (const auto& m : v.asObject()) members.push_back(&m);
        std::sort(members.begin(), members.end(), [](const auto* a, const auto* b) { return a->first < b->first; });
        out += '{';
        bool first = true;
        for (const auto* m : members) {
            if (!first) out += ',';
            first = false;
            writeString(out, m->first);
            out += ':';
            canonicalTo(out, m->second);
        }
        out += '}';
        break;
    }
    }
}

constexpr const char* kAlphabet = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

int sextet(char c) noexcept {
    if (c >= 'A' && c <= 'Z') return c - 'A';
    if (c >= 'a' && c <= 'z') return c - 'a' + 26;
    if (c >= '0' && c <= '9') return c - '0' + 52;
    if (c == '+') return 62;
    if (c == '/') return 63;
    return -1;
}

} // namespace

core::Json parseStrict(std::string_view text, std::string_view what) { return StrictParser(text, what).document(); }

std::string canonical(const core::Json& value) {
    std::string out;
    canonicalTo(out, value);
    return out;
}

std::optional<std::vector<std::uint8_t>> decodeBase64(std::string_view text) {
    if (text.size() % 4 != 0) return std::nullopt;
    std::vector<std::uint8_t> out;
    out.reserve(text.size() / 4 * 3);
    for (std::size_t i = 0; i < text.size(); i += 4) {
        const bool last = i + 4 == text.size();
        int v[4];
        int pad = 0;
        for (int k = 0; k < 4; ++k) {
            const char c = text[i + static_cast<std::size_t>(k)];
            if (c == '=' && last && k >= 2) {
                v[k] = 0, ++pad;
                continue;
            }
            if (pad) return std::nullopt; // ('=' only at the end)
            v[k] = sextet(c);
            if (v[k] < 0) return std::nullopt;
        }
        if (pad == 1 && text[i + 2] == '=') return std::nullopt;
        const unsigned n = (static_cast<unsigned>(v[0]) << 18) | (static_cast<unsigned>(v[1]) << 12) | (static_cast<unsigned>(v[2]) << 6) |
                           static_cast<unsigned>(v[3]);
        // canonical: the bits a padded group does not use are zero
        if (pad == 2 && (n & 0xFFFFu) != 0) return std::nullopt;
        if (pad == 1 && (n & 0xFFu) != 0) return std::nullopt;
        out.push_back(static_cast<std::uint8_t>(n >> 16));
        if (pad < 2) out.push_back(static_cast<std::uint8_t>((n >> 8) & 0xFF));
        if (pad < 1) out.push_back(static_cast<std::uint8_t>(n & 0xFF));
    }
    return out;
}

std::string encodeBase64(const std::uint8_t* data, std::size_t size) {
    std::string out;
    out.reserve((size + 2) / 3 * 4);
    for (std::size_t i = 0; i < size; i += 3) {
        const unsigned n = (static_cast<unsigned>(data[i]) << 16) | (i + 1 < size ? static_cast<unsigned>(data[i + 1]) << 8 : 0u) |
                           (i + 2 < size ? static_cast<unsigned>(data[i + 2]) : 0u);
        out += kAlphabet[(n >> 18) & 63];
        out += kAlphabet[(n >> 12) & 63];
        out += i + 1 < size ? kAlphabet[(n >> 6) & 63] : '=';
        out += i + 2 < size ? kAlphabet[n & 63] : '=';
    }
    return out;
}

std::optional<std::uint64_t> decimal(std::string_view text, std::uint64_t max) noexcept {
    if (text.empty() || text.size() > 20 || (text.size() > 1 && text[0] == '0')) return std::nullopt;
    std::uint64_t v = 0;
    for (const char c : text) {
        if (c < '0' || c > '9') return std::nullopt;
        const std::uint64_t d = static_cast<std::uint64_t>(c - '0');
        if (v > (max - d) / 10) return std::nullopt;
        v = v * 10 + d;
    }
    return v;
}

std::size_t codePoints(std::string_view text) noexcept {
    std::size_t n = 0;
    for (const char c : text)
        if ((static_cast<unsigned char>(c) & 0xC0) != 0x80) ++n;
    return n;
}

// --- reading against the contract -------------------------------------------------------------------------------------

void malformed(const std::string& path, const std::string& why) { throw Malformed(path + ": " + why); }

std::uint64_t idOf(const core::Json& v, const std::string& path, unsigned bits, bool nonzero) {
    if (!v.isString()) malformed(path, "an identity is an unsigned decimal string");
    const std::uint64_t max = bits >= 64 ? ~std::uint64_t{0} : (std::uint64_t{1} << bits) - 1;
    const std::optional<std::uint64_t> id = decimal(v.asString(), max);
    if (!id) malformed(path, "not an unsigned decimal of " + std::to_string(bits) + " bits without sign, space or leading zero");
    if (nonzero && *id == 0) malformed(path, "0 is not an identity here");
    return *id;
}

double numberOf(const core::Json& v, const std::string& path, Bounds b) {
    if (!v.isNumber()) malformed(path, "a number is required");
    const double d = v.asNumber();
    if (d < b.min || d > b.max || (b.exclusiveMin && d <= b.min)) malformed(path, "out of its range");
    return d;
}

std::int64_t integerOf(const core::Json& v, const std::string& path, std::int64_t min, std::int64_t max) {
    if (!v.isNumber()) malformed(path, "an integer is required");
    const double d = v.asNumber();
    if (std::floor(d) != d) malformed(path, "an integer is required");
    if (d < static_cast<double>(min) || d > static_cast<double>(max)) malformed(path, "out of its range");
    return static_cast<std::int64_t>(d);
}

bool boolOf(const core::Json& v, const std::string& path) {
    if (!v.isBool()) malformed(path, "true or false is required");
    return v.asBool();
}

const std::string& stringOf(const core::Json& v, const std::string& path, std::size_t maxLength, std::size_t minLength) {
    if (!v.isString()) malformed(path, "a string is required");
    const std::size_t n = codePoints(v.asString());
    if (n > maxLength || n < minLength) malformed(path, "a string of " + std::to_string(minLength) + " to " + std::to_string(maxLength) + " characters");
    return v.asString();
}

const std::string& boundedString(const core::Json& v, const std::string& path, std::size_t maxBytes) {
    const std::string& s = stringOf(v, path, maxBytes);
    if (s.size() > maxBytes) malformed(path, "longer than its native field's " + std::to_string(maxBytes) + " bytes");
    if (s.find('\0') != std::string::npos) malformed(path, "a NUL in a native text field");
    return s;
}

const std::string& nameOf(const core::Json& v, const std::string& path) {
    const std::string& s = stringOf(v, path, 63, 1);
    const auto ok = [](char c, bool first) {
        return (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || (!first && (c == '_' || c == '.' || c == '-'));
    };
    for (std::size_t i = 0; i < s.size(); ++i)
        if (!ok(s[i], i == 0)) malformed(path, "a name is [A-Za-z0-9][A-Za-z0-9_.-]*");
    return s;
}

bool lowerIdentifier(std::string_view text, std::size_t maxLength) noexcept {
    if (text.empty() || text.size() > maxLength || !(text[0] >= 'a' && text[0] <= 'z')) return false;
    for (const char c : text)
        if (!((c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '_')) return false;
    return true;
}

std::size_t enumOf(const core::Json& v, const std::string& path, std::initializer_list<const char*> names) {
    if (v.isString()) {
        std::size_t i = 0;
        for (const char* n : names) {
            if (v.asString() == n) return i;
            ++i;
        }
    }
    std::string list;
    for (const char* n : names) list += (list.empty() ? "" : ", ") + std::string(n);
    malformed(path, "one of " + list);
}

const core::Json::Array& arrayOf(const core::Json& v, const std::string& path, std::size_t minItems, std::size_t maxItems) {
    if (!v.isArray()) malformed(path, "an array is required");
    const std::size_t n = v.asArray().size();
    if (n < minItems || n > maxItems) malformed(path, std::to_string(minItems) + " to " + std::to_string(maxItems) + " items");
    return v.asArray();
}

Fields::Fields(const core::Json& value, std::string path) : path_(std::move(path)) {
    if (!value.isObject()) malformed(path_, "an object is required");
    object_ = &value.asObject();
    taken_.assign(object_->size(), false);
}

const core::Json* Fields::optional(std::string_view name) {
    for (std::size_t i = 0; i < object_->size(); ++i)
        if ((*object_)[i].first == name) {
            taken_[i] = true;
            return &(*object_)[i].second;
        }
    return nullptr;
}

const core::Json& Fields::required(std::string_view name) {
    const core::Json* v = optional(name);
    if (!v) malformed(path_, std::string(name) + " is required");
    return *v;
}

const core::Json* Fields::present(std::string_view name) {
    const core::Json* v = optional(name);
    return v && !v->isNull() ? v : nullptr;
}

bool Fields::has(std::string_view name) const noexcept {
    for (const auto& m : *object_)
        if (m.first == name) return true;
    return false;
}

void Fields::close() const {
    for (std::size_t i = 0; i < object_->size(); ++i)
        if (!taken_[i]) malformed(path_, "unknown field " + (*object_)[i].first);
}

void Writer::separate() {
    if (afterKey_) {
        afterKey_ = false;
        return;
    }
    if (!first_.empty()) {
        if (!first_.back()) out_ += ',';
        first_.back() = false;
    }
}
Writer& Writer::beginObject() {
    separate();
    out_ += '{';
    first_.push_back(true);
    return *this;
}
Writer& Writer::endObject() {
    out_ += '}';
    first_.pop_back();
    return *this;
}
Writer& Writer::beginArray() {
    separate();
    out_ += '[';
    first_.push_back(true);
    return *this;
}
Writer& Writer::endArray() {
    out_ += ']';
    first_.pop_back();
    return *this;
}
Writer& Writer::key(std::string_view name) {
    separate();
    writeString(out_, name);
    out_ += ':';
    afterKey_ = true;
    return *this;
}
Writer& Writer::string(std::string_view value) {
    separate();
    writeString(out_, value);
    return *this;
}
Writer& Writer::number(double value) {
    separate();
    writeNumber(out_, value);
    return *this;
}
Writer& Writer::integer(std::int64_t value) {
    separate();
    out_ += std::to_string(value);
    return *this;
}
Writer& Writer::boolean(bool value) {
    separate();
    out_ += value ? "true" : "false";
    return *this;
}
Writer& Writer::null() {
    separate();
    out_ += "null";
    return *this;
}
Writer& Writer::id(std::uint64_t value) {
    separate();
    out_ += '"';
    out_ += std::to_string(value);
    out_ += '"';
    return *this;
}
Writer& Writer::idOrNull(std::uint64_t value) { return value ? id(value) : null(); }
Writer& Writer::raw(std::string_view json) {
    separate();
    out_.append(json);
    return *this;
}
Writer& Writer::vector3(const double v[3]) {
    if (!std::isfinite(v[0]) || !std::isfinite(v[1]) || !std::isfinite(v[2])) return null();
    beginArray();
    for (int k = 0; k < 3; ++k) number(v[k]);
    return endArray();
}

} // namespace fsim::mission
