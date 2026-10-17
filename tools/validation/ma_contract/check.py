"""The mission-training contract's documents, held to their schema (docs/sdk/mission-training.md).

    python tools/validation/ma_contract/check.py [--repo <checkout>]

Checks, with the Python standard library alone:

  the schema    docs/sdk/ma-contract.schema.json uses only the JSON Schema 2020-12 keywords listed in KEYWORDS, every
                $ref resolves and every pattern compiles;
  the fixtures  each file tests/training/fixtures/mission_training/contract/manifest.json lists is parsed strictly (UTF-8, no
                NaN or Infinity, no duplicate key), validated against the schema, held to the rules a schema cannot
                state (rules()), and written out and read back unchanged - and fails exactly as its entry expects;
                together the valid ones must carry an identity above 2^53, an unknown numeric report and message
                bytes that are not UTF-8. A fixture is malformed (parse_refused, schema_invalid, rule_invalid: its
                document is refused whole) or valid, and a valid ma.actions.v1 may still hold requests the gateway
                refuses one by one (request_refusals: unsupported_kind, message_too_large);
  the pages     the contract's pages are printable ASCII, their local links and anchors resolve, and the request
                operations and gateway reasons mission-training.md tabulates are the schema's.

The SDK needs no schema library: this is a repository check, and the reference for what an adapter must enforce.
"""
import argparse
import base64
import json
import math
from pathlib import Path
import re
import sys

REPO = Path(__file__).resolve().parents[3]
SCHEMA = "docs/sdk/ma-contract.schema.json"
FIXTURES = "tests/training/fixtures/mission_training/contract"
CONTRACT = "docs/sdk/mission-training.md"
# Written for this contract: held to printable ASCII as well as to their links.
PAGES = (CONTRACT, "docs/sdk/ma-composition.md", "docs/sdk/ma-compatibility-baseline.md")
# Pages that lead to them: their local links are checked.
LINKING = ("docs/README.md", "docs/sdk/README.md", "docs/sdk/agents.md", "docs/architecture/design/agent-boundary.md", "docs/status.md",
           "docs/development.md",
           "docs/superpowers/specs/2026-10-12-ma-system-training-readiness-design.md",
           "docs/superpowers/plans/2026-10-12-ma-system-training-readiness.md",
           "docs/superpowers/plans/2026-10-12-ma-system-training-work-allocation.md")
# The dialect: the JSON Schema 2020-12 keywords this contract may use. Anything else in the schema is an error.
ANNOTATIONS = frozenset(("$schema", "$id", "$comment", "$defs", "title", "description"))
KEYWORDS = ANNOTATIONS | frozenset((
    "$ref", "type", "enum", "const", "pattern", "minLength", "maxLength", "minimum", "maximum", "exclusiveMinimum",
    "items", "minItems", "maxItems", "properties", "required", "additionalProperties", "propertyNames", "maxProperties",
    "allOf", "anyOf", "oneOf", "not", "if", "then"))
TYPES = ("null", "boolean", "integer", "number", "string", "array", "object")
# Defaults of ma.config.v1's limits (mission-training.md, "Capacity limits").
DEFAULT_LIMITS = {"completed_request_retention_ticks": 64, "max_in_flight_requests": 256, "max_requests_per_document": 64,
                  "max_document_bytes": 1048576, "max_message_bytes": 1024}
# The generic and noncombat request operations. A kind outside them is an extension's: refused unsupported_kind until
# its frozen definition and handler are installed.
GENERIC_KINDS = ("flight", "sensor", "dms", "task", "c2", "gateway")
U64 = re.compile(r"^(?:0|[1-9][0-9]{0,19})\Z")
# A receipt is closed once its state is final and open while it is queued; in between (accepted, pending, executing)
# it is open until its operation has nothing more to report or its service closes it.
CLOSED = {"queued": False, "rejected": True, "completed": True, "failed": True, "canceled": True}


MASK = (1 << 64) - 1


def advance(x):
    """fsim::Rng(x).next(): SplitMix64's step."""
    z = (x + 0x9E3779B97F4A7C15) & MASK
    z = ((z ^ (z >> 30)) * 0xBF58476D1CE4E5B9) & MASK
    z = ((z ^ (z >> 27)) * 0x94D049BB133111EB) & MASK
    return z ^ (z >> 31)


def stream_seed(base_seed, env_id, local_episode_index, stream):
    """An episode's seed for one stream class (mission-training.md, "Seed streams"): 0 its World's, 1 its services'."""
    return advance(advance(advance(advance(base_seed) ^ env_id) ^ local_episode_index) ^ stream)


def service_seed(base_seed, env_id, local_episode_index, service_id):
    return advance(stream_seed(base_seed, env_id, local_episode_index, 1) ^ service_id)


def component_seed(base_seed, env_id, local_episode_index, service_id, component_id):
    return advance(service_seed(base_seed, env_id, local_episode_index, service_id) ^ component_id)


def check_seeds(vectors):
    errors = []
    for i, v in enumerate(vectors):
        key = [int(v[k]) for k in ("base_seed", "env_id", "local_episode_index")]
        found = {"world_seed": stream_seed(*key, 0),
                 "service_seed": service_seed(*key, int(v["service_id"])),
                 "component_seed": component_seed(*key, int(v["service_id"]), int(v["component_id"]))}
        for name, value in found.items():
            if v[name] != str(value):
                errors.append("seed_streams[%d]: %s is %s, the derivation gives %d" % (i, name, v[name], value))
    return errors


class Refused(ValueError):
    """A document that is not strict JSON."""


def parse(data):
    """A document's bytes as strict JSON: UTF-8 without a byte order mark, no NaN or Infinity, no duplicate key."""
    if isinstance(data, bytes):
        if data.startswith(b"\xef\xbb\xbf"):
            raise Refused("a byte order mark")
        try:
            data = data.decode("utf-8")
        except UnicodeDecodeError as e:
            raise Refused("not UTF-8: %s" % e)

    def constant(name):
        raise Refused("%s is not JSON" % name)

    def pairs(items):
        out = {}
        for key, value in items:
            if key in out:
                raise Refused("duplicate key %r" % key)
            out[key] = value
        return out

    def number(text):
        value = float(text)
        if not math.isfinite(value):
            raise Refused("%s is not a finite number" % text)
        return value
    try:
        return json.loads(data, parse_constant=constant, object_pairs_hook=pairs, parse_float=number)
    except json.JSONDecodeError as e:
        raise Refused("not JSON: %s" % e)


def dump(document):
    """Canonical text: keys sorted, no insignificant space, numbers in their shortest round-trip form."""
    return json.dumps(document, sort_keys=True, separators=(",", ":"), ensure_ascii=False, allow_nan=False)


def kind(value):
    if value is None:
        return "null"
    if isinstance(value, bool):
        return "boolean"
    if isinstance(value, int):
        return "integer"
    if isinstance(value, float):
        return "integer" if value.is_integer() else "number"
    return {str: "string", list: "array", dict: "object"}[type(value)]


def same(a, b):
    """JSON equality: true is not 1."""
    if isinstance(a, bool) or isinstance(b, bool):
        return isinstance(a, bool) and isinstance(b, bool) and a == b
    if isinstance(a, (int, float)) and isinstance(b, (int, float)):
        return a == b
    if type(a) is not type(b):
        return False
    if isinstance(a, list):
        return len(a) == len(b) and all(same(x, y) for x, y in zip(a, b))
    if isinstance(a, dict):
        return a.keys() == b.keys() and all(same(a[k], b[k]) for k in a)
    return a == b


def pattern(text, cache={}):
    """A schema's pattern, as ECMA-262 reads it: its closing $ is the end of the string, never before a last line
    break as Python's is."""
    if text not in cache:
        cache[text] = re.compile(text[:-1] + r"\Z" if text.endswith("$") and not text.endswith(r"\$") else text)
    return cache[text]


class Schema:
    def __init__(self, document):
        self.root = document
        self.errors = []
        self._walk(document, "#")

    def _walk(self, node, where):
        """The dialect: only KEYWORDS, each used as 2020-12 defines it."""
        if isinstance(node, bool):
            return
        if not isinstance(node, dict):
            self.errors.append("%s: a schema is an object or a boolean" % where)
            return
        for key, value in node.items():
            here = "%s/%s" % (where, key)
            if key not in KEYWORDS:
                self.errors.append("%s: keyword outside the contract's dialect" % here)
            elif key in ("properties", "$defs"):
                for name, sub in value.items():
                    self._walk(sub, "%s/%s" % (here, name))
            elif key in ("items", "additionalProperties", "propertyNames", "not", "if", "then"):
                self._walk(value, here)
            elif key in ("allOf", "anyOf", "oneOf"):
                for i, sub in enumerate(value):
                    self._walk(sub, "%s/%d" % (here, i))
            elif key == "$ref":
                try:
                    self.resolve(value)
                except KeyError:
                    self.errors.append("%s: %s does not resolve" % (here, value))
            elif key == "pattern":
                if not (value.startswith("^") and value.endswith("$") and not value.endswith(r"\$")):
                    self.errors.append("%s: a pattern is anchored at both ends" % here)
                try:
                    pattern(value)
                except re.error as e:
                    self.errors.append("%s: %s" % (here, e))
            elif key == "type":
                for t in ([value] if isinstance(value, str) else value):
                    if t not in TYPES:
                        self.errors.append("%s: %r is not a type" % (here, t))
        if "then" in node and "if" not in node:
            self.errors.append("%s: then without if" % where)

    def resolve(self, ref):
        if not ref.startswith("#/"):
            raise KeyError(ref)
        node = self.root
        for part in ref[2:].split("/"):
            node = node[part]
        return node

    def validate(self, value, schema=None, where="$"):
        """The reasons `value` is not what `schema` (the root's) describes; none when it is."""
        schema = self.root if schema is None else schema
        if schema is True:
            return []
        if schema is False:
            return ["%s: not allowed here" % where]
        errors = []
        if "$ref" in schema:
            errors += self.validate(value, self.resolve(schema["$ref"]), where)
        if "type" in schema:
            types = [schema["type"]] if isinstance(schema["type"], str) else schema["type"]
            actual = kind(value)
            if actual not in types and not (actual == "integer" and "number" in types):
                errors.append("%s: %s, not %s" % (where, actual, " or ".join(types)))
        if "enum" in schema and not any(same(value, e) for e in schema["enum"]):
            errors.append("%s: %s is not one of %s" % (where, json.dumps(value), json.dumps(schema["enum"])))
        if "const" in schema and not same(value, schema["const"]):
            errors.append("%s: %s is not %s" % (where, json.dumps(value), json.dumps(schema["const"])))
        if isinstance(value, str):
            if "pattern" in schema and not pattern(schema["pattern"]).search(value):
                errors.append("%s: %s does not match its pattern" % (where, json.dumps(value[:40])))
            if len(value) < schema.get("minLength", 0):
                errors.append("%s: shorter than %d" % (where, schema["minLength"]))
            if "maxLength" in schema and len(value) > schema["maxLength"]:
                errors.append("%s: longer than %d" % (where, schema["maxLength"]))
        if isinstance(value, (int, float)) and not isinstance(value, bool):
            if "minimum" in schema and value < schema["minimum"]:
                errors.append("%s: %s is below %s" % (where, value, schema["minimum"]))
            if "maximum" in schema and value > schema["maximum"]:
                errors.append("%s: %s is above %s" % (where, value, schema["maximum"]))
            if "exclusiveMinimum" in schema and value <= schema["exclusiveMinimum"]:
                errors.append("%s: %s is not above %s" % (where, value, schema["exclusiveMinimum"]))
        if isinstance(value, list):
            if len(value) < schema.get("minItems", 0):
                errors.append("%s: fewer than %d items" % (where, schema["minItems"]))
            if "maxItems" in schema and len(value) > schema["maxItems"]:
                errors.append("%s: more than %d items" % (where, schema["maxItems"]))
            if "items" in schema:
                for i, item in enumerate(value):
                    errors += self.validate(item, schema["items"], "%s[%d]" % (where, i))
        if isinstance(value, dict):
            properties = schema.get("properties", {})
            for name in schema.get("required", []):
                if name not in value:
                    errors.append("%s: %s is required" % (where, name))
            if "maxProperties" in schema and len(value) > schema["maxProperties"]:
                errors.append("%s: more than %d members" % (where, schema["maxProperties"]))
            for name, member in value.items():
                if name in properties:
                    errors += self.validate(member, properties[name], "%s.%s" % (where, name))
                elif "additionalProperties" in schema:
                    if schema["additionalProperties"] is False:
                        errors.append("%s: unknown field %s" % (where, name))
                    else:
                        errors += self.validate(member, schema["additionalProperties"], "%s.%s" % (where, name))
                if "propertyNames" in schema:
                    errors += self.validate(name, schema["propertyNames"], "%s (the name %s)" % (where, name))
        for sub in schema.get("allOf", []):
            errors += self.validate(value, sub, where)
        if "anyOf" in schema and all(self.validate(value, sub, where) for sub in schema["anyOf"]):
            errors.append("%s: matches none of its alternatives" % where)
        if "oneOf" in schema:
            results = [self.validate(value, sub, where) for sub in schema["oneOf"]]
            passed = sum(1 for r in results if not r)
            if passed != 1:
                nearest = min(results, key=len) if passed == 0 else []
                errors.append("%s: matches %d of its alternatives, not one%s" % (where, passed, "".join("; " + e for e in nearest[:3])))
        if "not" in schema and not self.validate(value, schema["not"], where):
            errors.append("%s: is what it may not be" % where)
        if "if" in schema and "then" in schema and not self.validate(value, schema["if"], where):
            errors += self.validate(value, schema["then"], where)
        return errors


def decoded(text, where, errors):
    """A base64 field's bytes, canonical or refused."""
    try:
        data = base64.b64decode(text, validate=True)
    except ValueError as e:
        errors.append("%s: not base64: %s" % (where, e))
        return None
    if base64.b64encode(data).decode("ascii") != text:
        errors.append("%s: base64 is not canonical (unused bits set)" % where)
        return None
    return data


def message_bytes(holder, where, errors):
    """A message's bytes, or the reason its document is malformed: base64 that is not canonical, or a byte_length that
    is not the decoded length. Its size against a configured limit is not this: see refusals()."""
    data = decoded(holder["data_b64"], where + ".data_b64", errors)
    if data is None:
        return None
    if len(data) != holder["byte_length"]:
        errors.append("%s: byte_length %d, but %d bytes decoded" % (where, holder["byte_length"], len(data)))
    return data


def refusals(document, limits=None):
    """The requests of a well-formed ma.actions.v1 that the gateway refuses one by one at submit, as far as the
    document and its limits alone show: {request_id: gateway reason}. The document itself is accepted, its other
    requests are queued, and each refused request gets a closed, rejected receipt."""
    limits = dict(DEFAULT_LIMITS, **(limits or {}))
    refused = {}
    for request in document["requests"] if document.get("schema") == "ma.actions.v1" else []:
        if request["kind"] not in GENERIC_KINDS:
            refused[request["request_id"]] = "unsupported_kind"
        elif request["kind"] == "dms" and request["operation"] == "send":
            data = decoded(request["parameters"]["data_b64"], "", [])
            if data is not None and len(data) > limits["max_message_bytes"]:
                refused[request["request_id"]] = "message_too_large"
    return refused


def platforms(config):
    """The names a configuration's scenario and C2 nodes give its platforms (a vehicle with a count: "<name>-<n>")."""
    names = []
    for vehicle in config["scenario"].get("vehicles", []):
        count = vehicle.get("count", 1)
        names += [vehicle["name"]] if count == 1 else ["%s-%d" % (vehicle["name"], i + 1) for i in range(count)]
    return names + [node["name"] for node in config.get("c2_nodes", [])]


def config_rules(config, errors):
    names = platforms(config)
    for name in sorted(set(n for n in names if names.count(n) > 1)):
        errors.append("$: platform %s is named twice" % name)

    def known(name, where):
        if name not in names:
            errors.append("%s: %s is no platform of the scenario or its C2 nodes" % (where, name))
    for section in ("systems", "dissemination", "rbac"):
        seen = set()
        for i, entry in enumerate(config.get(section, [])):
            known(entry["platform"], "$.%s[%d]" % (section, i))
            if entry["platform"] in seen:
                errors.append("$.%s[%d]: %s is configured twice" % (section, i, entry["platform"]))
            seen.add(entry["platform"])
    ids = [s["system_id"] for s in config.get("systems", [])]
    if len(ids) != len(set(ids)):
        errors.append("$.systems: a system_id is assigned twice")
    for i, net in enumerate(config.get("nets", [])):
        for member in net["members"]:
            known(member, "$.nets[%d]" % i)
        if len(net["members"]) != len(set(net["members"])):
            errors.append("$.nets[%d]: a member is listed twice" % i)
    nets = [net["name"] for net in config.get("nets", [])]
    if len(nets) != len(set(nets)):
        errors.append("$.nets: a net is named twice")
    services = config["services"]
    for field in ("service_id", "name"):
        values = [s[field] for s in services]
        if len(values) != len(set(values)):
            errors.append("$.services: a %s is used twice" % field)
    bound = {}
    for i, service in enumerate(services):
        known(service["platform"], "$.services[%d]" % i)
        bound.setdefault(service["platform"], []).append(service)
        capabilities = service["command_capabilities"]
        if len(capabilities) != len(set(capabilities)):
            errors.append("$.services[%d]: a command capability is listed twice" % i)
        if "*" in capabilities and len(capabilities) != 1:
            errors.append("$.services[%d]: \"*\" stands alone" % i)
    for platform, group in sorted(bound.items()):
        for owner in ("owns_dms_ingress", "owns_dms_egress"):
            count = sum(1 for s in group if s[owner])
            if count != 1:
                errors.append("$.services: %s has %d services with %s, not one" % (platform, count, owner))
        held = [c for s in group for c in s["command_capabilities"]]
        if "*" in held and len(held) != 1:
            errors.append("$.services: %s gives every command capability to one service and some to another" % platform)
        for capability in sorted(set(c for c in held if held.count(c) > 1)):
            errors.append("$.services: %s gives %s to two services" % (platform, capability))
    criteria = config.get("task", {}).get("criteria", [])
    if len(criteria) != len(set(c["id"] for c in criteria)):
        errors.append("$.task.criteria: an id is used twice")
    for i, criterion in enumerate(criteria):
        for field in ("platform", "from_platform"):
            if field in criterion:
                known(criterion[field], "$.task.criteria[%d]" % i)
        if criterion.get("altitude_min_m", -math.inf) > criterion.get("altitude_max_m", math.inf):
            errors.append("$.task.criteria[%d]: altitude_min_m is above altitude_max_m" % i)


def receipt_rules(receipts, where, errors):
    ids = [r["request_id"] for r in receipts]
    if len(ids) != len(set(ids)):
        errors.append("%s: a request has two receipts" % where)
    if ids != sorted(ids, key=int):
        errors.append("%s: receipts are not in request-id order" % where)
    for i, receipt in enumerate(receipts):
        here = "%s[%d]" % (where, i)
        closed = CLOSED.get(receipt["state"])
        if closed is not None and receipt["closed"] != closed:
            errors.append("%s: state %s is %s" % (here, receipt["state"], "closed" if closed else "not closed"))
        if receipt["gateway_reason"] is not None and (receipt["state"] != "rejected" or receipt["native"] is not None):
            errors.append("%s: a gateway refusal is rejected, with no native answer" % here)
        if receipt["state"] == "queued" and receipt["native"] is not None:
            errors.append("%s: a queued request has no native answer yet" % here)
        if receipt["kind"] == "gateway":
            # gateway.close is the transport's own act, with no simulation call and so no native answer (section 12):
            # queued; rejected by the gateway; or accepted and closed - nothing else
            state, closed = receipt["state"], receipt["closed"]
            if (receipt["operation"] != "close" or receipt["native"] is not None or
                    not ((state == "queued" and not closed) or (state == "accepted" and closed) or
                         (state == "rejected" and closed and receipt["gateway_reason"] is not None))):
                errors.append("%s: a gateway.close receipt is queued, rejected by the gateway, or accepted and closed - "
                              "with no native answer" % here)
        elif receipt["state"] not in ("queued", "rejected") and receipt["native"] is None:
            errors.append("%s: state %s without the native answer it stands for" % (here, receipt["state"]))
        if int(receipt["updated_tick"]) < int(receipt["submitted_tick"]):
            errors.append("%s: updated before it was submitted" % here)


def rules(document, limits=None):
    """What v1 requires of a schema-valid document beyond its schema: (why it is malformed, its request operations no
    generic definition handles). A malformed document is refused whole, as an error of the call. A request that is
    merely refused - refusals() - leaves its document well-formed."""
    limits = dict(DEFAULT_LIMITS, **(limits or {}))
    errors, unsupported = [], []
    name = document["schema"]
    if name == "ma.config.v1":
        config_rules(document, errors)
    elif name == "ma.actions.v1":
        requests = document["requests"]
        if len(requests) > limits["max_requests_per_document"]:
            errors.append("$.requests: %d requests, over the %d the configuration allows" % (len(requests), limits["max_requests_per_document"]))
        ids = [int(r["request_id"]) for r in requests]
        if any(b <= a for a, b in zip(ids, ids[1:])):
            errors.append("$.requests: request ids do not strictly increase")
        for i, request in enumerate(requests):
            if request["kind"] not in GENERIC_KINDS:
                unsupported.append("%s.%s" % (request["kind"], request["operation"]))
            elif request["kind"] == "dms" and request["operation"] == "send":
                message_bytes(request["parameters"], "$.requests[%d].parameters" % i, errors)
    elif name == "ma.observation.v1":
        now = document["time_s"]
        for key, report in document["reports"].items():
            here = "$.reports.%s" % key
            items = report.get("items")
            if items is not None:
                if report["count"] != len(items):
                    errors.append("%s: count %d, but %d items" % (here, report["count"], len(items)))
                if report["availability"] != "available" and items:
                    errors.append("%s: %s, yet it lists items" % (here, report["availability"]))
            metas = [(here, report["meta"])] if "meta" in report else []
            metas += [("%s.items[%d]" % (here, i), item["meta"]) for i, item in enumerate(items or []) if "meta" in item]
            for where, meta in metas:
                sample, age = meta["sample_time_s"], meta["age_s"]
                if (sample is None) != (age is None):
                    errors.append("%s.meta: a sample time and its age are known together" % where)
                elif sample is not None and abs(now - sample - age) > 1e-6:
                    errors.append("%s.meta: age_s is not time_s less sample_time_s" % where)
                if meta["validity"] == "stale" and age is None:
                    errors.append("%s.meta: stale without an age" % where)
        navigation = document["reports"]["navigation"]
        if (navigation["availability"] == "available") != (navigation["own_ship"] is not None):
            errors.append("$.reports.navigation: own_ship is given exactly when it is available")
        messages = document["messages"]
        if messages["count"] != len(messages["items"]):
            errors.append("$.messages: count %d, but %d items" % (messages["count"], len(messages["items"])))
        if messages["availability"] != "available" and messages["items"]:
            errors.append("$.messages: %s, yet it lists items" % messages["availability"])
        for i, message in enumerate(messages["items"]):
            message_bytes(message, "$.messages.items[%d]" % i, errors)  # (a delivered message's size is its net's)
        receipt_rules(document["receipts"], "$.receipts", errors)
    elif name == "ma.receipts.v1":
        receipt_rules(document["receipts"], "$.receipts", errors)
        submission = document["submission"]
        if submission is not None:
            if submission["accepted"] != (submission["reason"] is None):
                errors.append("$.submission: accepted exactly when it gives no reason")
            if not submission["accepted"] and submission["requests"]:
                errors.append("$.submission: a refused document queued nothing")
            ids = [int(r["request_id"]) for r in submission["requests"]]
            if any(b <= a for a, b in zip(ids, ids[1:])):
                errors.append("$.submission.requests: request ids do not strictly increase, as its document's did")
            for i, request in enumerate(submission["requests"]):
                if (request["status"] == "rejected") != (request["reason"] is not None):
                    errors.append("$.submission.requests[%d]: a reason exactly when rejected" % i)
    elif name == "ma.outcome.v1":
        satisfied = {o: [c for c in document["criteria"] if c["outcome"] == o and c["satisfied"]] for o in ("success", "failure", "truncated")}
        successes = [c for c in document["criteria"] if c["outcome"] == "success"]
        expected = ("failure" if satisfied["failure"] else "success" if successes and len(satisfied["success"]) == len(successes)
                    else "truncated" if satisfied["truncated"] else None)
        if document["reason"] != expected or document["episode_done"] != (expected is not None):
            errors.append("$: reason %s, but its criteria give %s" % (json.dumps(document["reason"]), json.dumps(expected)))
        for i, criterion in enumerate(document["criteria"]):
            if criterion["satisfied"] != (criterion["satisfied_tick"] is not None):
                errors.append("$.criteria[%d]: a tick exactly when satisfied" % i)
        for i, role in enumerate(document["roles"]):
            if role["active"] and (role["terminated"] or role["truncated"]):
                errors.append("$.roles[%d]: active, yet ended" % i)
            if role["terminated"] and role["truncated"]:
                errors.append("$.roles[%d]: terminated and truncated at once" % i)
            if role["bootstrap"] != role["truncated"]:
                errors.append("$.roles[%d]: bootstrap exactly when truncated" % i)
            if role["final_observation"] != (role["terminated"] or role["truncated"]):
                errors.append("$.roles[%d]: a final observation exactly when it ended" % i)
    return errors, unsupported


def strings(node):
    if isinstance(node, str):
        yield node
    elif isinstance(node, list):
        for item in node:
            yield from strings(item)
    elif isinstance(node, dict):
        for value in node.values():
            yield from strings(value)


def coverage(document, found):
    """What M0 requires the valid fixtures to carry between them."""
    if any(U64.match(s) and int(s) > 2 ** 53 for s in strings(document)):
        found.add("identity_above_2_53")
    holders = []
    if document["schema"] == "ma.observation.v1":
        holders = document["messages"]["items"]
        text = dump(document["reports"])
        if ":null" in text and ('"validity":"unknown"' in text or '"availability":"unavailable"' in text):
            found.add("unknown_numeric_report")
    elif document["schema"] == "ma.actions.v1":
        holders = [r["parameters"] for r in document["requests"] if r["kind"] == "dms" and r["operation"] == "send"]
    for holder in holders:
        try:
            base64.b64decode(holder["data_b64"], validate=True).decode("utf-8")
        except UnicodeDecodeError:
            found.add("non_utf8_message_bytes")
        except ValueError:
            pass


def check_fixture(schema, path, entry):
    """Why the fixture is not what its manifest entry expects; and, valid, its document."""
    expect = entry["expect"]
    try:
        document = parse(path.read_bytes())
    except Refused as e:
        return ([] if expect == "parse_refused" else ["refused as JSON (%s), expected %s" % (e, expect)]), None
    if expect == "parse_refused":
        return ["parsed, expected to be refused as JSON"], None
    found = schema.validate(document)
    if found:
        wanted = entry.get("error_contains", "")
        if expect != "schema_invalid":
            return ["schema: " + e for e in found[:6]], None
        return ([] if any(wanted in e for e in found) else ["schema-invalid, but not for %r: %s" % (wanted, found[0])]), None
    if expect == "schema_invalid":
        return ["schema-valid, expected schema_invalid"], None
    found, unsupported = rules(document, entry.get("limits"))
    if expect == "rule_invalid":
        wanted = entry.get("error_contains", "")
        return ([] if any(wanted in e for e in found) else ["expected a rule to refuse it for %r: %s" % (wanted, found[:2])]), None
    errors = ["rule: " + e for e in found]
    refused = refusals(document, entry.get("limits"))
    if refused != entry.get("request_refusals", {}):
        errors.append("request refusals %s, expected %s" % (json.dumps(refused, sort_keys=True),
                                                           json.dumps(entry.get("request_refusals", {}), sort_keys=True)))
    if expect != "valid":
        errors.append("unknown expectation %r" % expect)
    if parse(dump(document).encode("utf-8")) != document or not same(parse(dump(document)), document):
        errors.append("does not read back as it was written")
    return errors, document


def check_fixtures(repo, schema):
    directory = repo / FIXTURES
    manifest = parse((directory / "manifest.json").read_bytes())
    errors, found, counts = [], set(), {}
    listed = set()
    for entry in manifest["fixtures"]:
        listed.add(entry["file"])
        path = directory / entry["file"]
        if not path.is_file():
            errors.append("%s: listed, not there" % entry["file"])
            continue
        problems, document = check_fixture(schema, path, entry)
        errors += ["%s: %s" % (entry["file"], p) for p in problems]
        counts[entry["expect"]] = counts.get(entry["expect"], 0) + 1
        if document is not None and entry["expect"] == "valid":
            coverage(document, found)
    for path in sorted(directory.glob("*.json")):
        if path.name != "manifest.json" and path.name not in listed:
            errors.append("%s: not in the manifest" % path.name)
    for required in manifest["requires"]:
        if required not in found:
            errors.append("no valid fixture carries %s" % required)
    errors += check_seeds(manifest["seed_streams"])
    if not manifest["seed_streams"]:
        errors.append("the manifest gives no seed stream vector")
    return errors, counts


LINK = re.compile(r"(?<!\!)\[[^\]\n]*\]\(([^)\s]+)\)")
HEADING = re.compile(r"^#{1,6}\s+(.*?)\s*#*\s*$")


def anchors(text):
    """A Markdown page's heading anchors, as GitHub derives them."""
    out, seen, fenced = set(), {}, False
    for line in text.splitlines():
        if line.lstrip().startswith("```"):
            fenced = not fenced
        m = None if fenced else HEADING.match(line)
        if not m:
            continue
        title = re.sub(r"\[([^\]]*)\]\([^)]*\)", r"\1", m.group(1)).replace("`", "")
        slug = re.sub(r"[^\w\- ]", "", title.lower()).replace(" ", "-")
        n = seen.get(slug, 0)
        seen[slug] = n + 1
        out.add(slug if n == 0 else "%s-%d" % (slug, n))
    return out


def check_pages(repo, schema_document):
    errors, links = [], 0
    for relative in PAGES + LINKING:
        path = repo / relative
        if not path.is_file():
            errors.append("%s: missing" % relative)
            continue
        text = path.read_text(encoding="utf-8")
        if relative in PAGES:
            for number, line in enumerate(text.splitlines(), 1):
                bad = [c for c in line if not (" " <= c <= "~" or c == "\t")]
                if bad:
                    errors.append("%s:%d: not printable ASCII: %r" % (relative, number, bad[0]))
        fenced = False
        for number, line in enumerate(text.splitlines(), 1):
            if line.lstrip().startswith("```"):
                fenced = not fenced
            if fenced:
                continue
            for target in LINK.findall(line):
                if re.match(r"[a-z][a-z0-9+.-]*:", target):
                    continue  # (another site's, or a mail address)
                links += 1
                file, _, anchor = target.partition("#")
                linked = path if not file else (path.parent / file).resolve()
                if not linked.exists():
                    errors.append("%s:%d: %s does not exist" % (relative, number, target))
                elif anchor and linked.suffix == ".md" and anchor not in anchors(linked.read_text(encoding="utf-8")):
                    errors.append("%s:%d: %s has no heading #%s" % (relative, number, file or relative, anchor))
    contract = repo / CONTRACT
    if contract.is_file():
        text = contract.read_text(encoding="utf-8")
        defs = schema_document["$defs"]
        def section(title):
            """The page's section whose heading holds `title`, to the next heading of its level."""
            m = re.search(r"^## [^\n]*%s[^\n]*\n(.*?)(?=^## |\Z)" % re.escape(title), text, re.M | re.S)
            return m.group(1) if m else ""
        operations = {"%s.%s" % (k, op) for k in GENERIC_KINDS for op in defs["request_" + k]["properties"]["operation"]["enum"]}
        tabulated = set(re.findall(r"^\| `([a-z0-9_]+\.[a-z0-9_]+)` \|", section("`ma.actions.v1`"), re.M))
        for name in sorted(operations ^ tabulated):
            errors.append("%s: request operation %s is in %s only" % (CONTRACT, name, "the schema" if name in operations else "the page"))
        reasons = set(defs["gateway_reason"]["enum"])
        tabulated = set(re.findall(r"^\| `([a-z_]+)` \| (?:submit|dispatch)", section("`ma.receipts.v1`"), re.M))
        for name in sorted(reasons ^ tabulated):
            errors.append("%s: gateway reason %s is in %s only" % (CONTRACT, name, "the schema" if name in reasons else "the page"))
        for document in sorted(s["properties"]["schema"]["const"] for s in defs.values()
                               if isinstance(s, dict) and "schema" in s.get("properties", {})):
            if "`%s`" % document not in text:
                errors.append("%s: document %s is not described" % (CONTRACT, document))
    return errors, links


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--repo", default=str(REPO))
    a = ap.parse_args()
    repo = Path(a.repo).resolve()
    document = parse((repo / SCHEMA).read_bytes())
    schema = Schema(document)
    errors = ["schema %s" % e for e in schema.errors]
    if document.get("$schema") != "https://json-schema.org/draft/2020-12/schema":
        errors.append("schema: its dialect is not JSON Schema 2020-12")
    fixture_errors, counts = check_fixtures(repo, schema)
    page_errors, links = check_pages(repo, document)
    errors += fixture_errors + page_errors
    for e in errors:
        print(e)
    print("%d definitions; %d fixtures (%s); %d pages, %d local links; %d errors" % (
        len(document["$defs"]), sum(counts.values()), ", ".join("%d %s" % (n, k) for k, n in sorted(counts.items())),
        len(PAGES + LINKING), links, len(errors)))
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main())
