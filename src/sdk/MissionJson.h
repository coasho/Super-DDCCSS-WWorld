#pragma once

// The mission documents' text (docs/sdk/mission-training.md, section 6): a strict RFC 8259 reader for what comes in,
// a writer for what goes out, canonical base64 and the contract's decimal identities. Internal to the SDK.
//
// core::Json holds a parsed document - insertion order kept, numbers as doubles, identities as the strings they are -
// but its own parser is lenient by design; parseStrict() is the reading the contract asks for: UTF-8 without a byte
// order mark, no comment, trailing comma, duplicate member, NaN, Infinity or number a double cannot hold.

#include "foundation/core/Json.h"

#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

// A reader returns a reference into the document it is given, never into its path: GCC's dangling-reference heuristic,
// seeing a temporary path, cannot tell.
#if defined(__GNUC__) && !defined(__clang__) && __GNUC__ >= 14
#define FSIM_MISSION_NO_DANGLING [[gnu::no_dangling]]
#else
#define FSIM_MISSION_NO_DANGLING
#endif

namespace fsim::mission {

/// A document the contract calls malformed: refused whole, an error of the call.
class Malformed : public std::invalid_argument {
public:
    explicit Malformed(const std::string& what) : std::invalid_argument(what) {}
};

/// Strict JSON text of one object; `what` names the document in the error ("ma.actions.v1: ...").
core::Json parseStrict(std::string_view text, std::string_view what);

/// The canonical text of a value, for comparing bodies: members sorted by name, numbers in their shortest
/// round-trip form, no insignificant space.
std::string canonical(const core::Json& value);

/// Canonical RFC 4648 base64 (standard alphabet, padded, no line break, unused bits zero), or nothing.
std::optional<std::vector<std::uint8_t>> decodeBase64(std::string_view text);
std::string encodeBase64(const std::uint8_t* data, std::size_t size);

/// An unsigned decimal identity without sign, space or leading zero, at most `max`; nothing otherwise.
std::optional<std::uint64_t> decimal(std::string_view text, std::uint64_t max) noexcept;
/// Code points of UTF-8 text already checked to be UTF-8 (a schema's lengths count them).
std::size_t codePoints(std::string_view text) noexcept;

// --- Reading a parsed document against the contract -----------------------------------------------------------------
//
// Each reader takes a value and its path ("ma.actions.v1: $.requests[2].parameters.mode") and returns it in native form,
// or throws Malformed naming the path and the rule it breaks.

[[noreturn]] void malformed(const std::string& path, const std::string& why);

/// A number's bounds, as the schema states them.
struct Bounds {
    double min = -std::numeric_limits<double>::infinity(), max = std::numeric_limits<double>::infinity();
    bool exclusiveMin = false;
};
inline Bounds atLeast(double min) { return Bounds{min, std::numeric_limits<double>::infinity(), false}; }
inline Bounds above(double min) { return Bounds{min, std::numeric_limits<double>::infinity(), true}; }
inline Bounds within(double min, double max) { return Bounds{min, max, false}; }

std::uint64_t idOf(const core::Json& v, const std::string& path, unsigned bits = 64, bool nonzero = false);
double numberOf(const core::Json& v, const std::string& path, Bounds bounds = {});
std::int64_t integerOf(const core::Json& v, const std::string& path, std::int64_t min, std::int64_t max);
bool boolOf(const core::Json& v, const std::string& path);
/// A string of at most `maxLength` code points (and at least `minLength`).
FSIM_MISSION_NO_DANGLING const std::string& stringOf(const core::Json& v, const std::string& path, std::size_t maxLength, std::size_t minLength = 0);
/// The same, also at most `maxBytes` bytes: a native fixed field less its terminator (section 14).
FSIM_MISSION_NO_DANGLING const std::string& boundedString(const core::Json& v, const std::string& path, std::size_t maxBytes);
/// A `name`: ^[A-Za-z0-9][A-Za-z0-9_.-]*$, 1 to 63.
FSIM_MISSION_NO_DANGLING const std::string& nameOf(const core::Json& v, const std::string& path);
/// A lower-case identifier: ^[a-z][a-z0-9_]*$, 1 to `maxLength`.
bool lowerIdentifier(std::string_view text, std::size_t maxLength) noexcept;
/// The index of the value among `names`.
std::size_t enumOf(const core::Json& v, const std::string& path, std::initializer_list<const char*> names);
FSIM_MISSION_NO_DANGLING const core::Json::Array& arrayOf(const core::Json& v, const std::string& path, std::size_t minItems, std::size_t maxItems);

/// One object of a document: its members taken by name, and - a closed object - any other refused.
class Fields {
public:
    Fields(const core::Json& value, std::string path);
    const core::Json* optional(std::string_view name);   ///< null when absent
    const core::Json& required(std::string_view name);
    /// The member, or nothing when it is absent or null (null in a request means the same as leaving it out).
    const core::Json* present(std::string_view name);
    bool has(std::string_view name) const noexcept;
    std::string path(std::string_view name) const { return path_ + "." + std::string(name); }
    const std::string& path() const noexcept { return path_; }
    std::size_t size() const noexcept { return taken_.size(); }
    /// Every member not taken is an unknown field.
    void close() const;

private:
    const core::Json::Object* object_ = nullptr;
    std::string path_;
    std::vector<bool> taken_;
};

/// Compact JSON text, written in order; commas and the member separator are its own business.
class Writer {
public:
    Writer& beginObject();
    Writer& endObject();
    Writer& beginArray();
    Writer& endArray();
    Writer& key(std::string_view name);
    Writer& string(std::string_view value);
    Writer& number(double value);                    ///< null unless finite; shortest round-trip form
    Writer& integer(std::int64_t value);
    Writer& boolean(bool value);
    Writer& null();
    Writer& id(std::uint64_t value);                 ///< an identity: its decimal string
    Writer& idOrNull(std::uint64_t value);           ///< 0: null
    Writer& raw(std::string_view json);              ///< a value already written
    Writer& vector3(const double v[3]);              ///< null unless all three are finite
    Writer& member(std::string_view name, std::string_view value) { return key(name).string(value); }
    std::string take() { return std::move(out_); }
    const std::string& text() const noexcept { return out_; }

private:
    void separate();
    std::string out_;
    std::vector<bool> first_; ///< per open container: nothing written in it yet
    bool afterKey_ = false;
};

} // namespace fsim::mission
