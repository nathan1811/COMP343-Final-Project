import json
import os
import re
from typing import Any, Dict, List, Optional

from pydantic import BaseModel, Field, ValidationError

OPENAI_MODEL = "gpt-5-mini"
DEFAULT_DRAIN = 3.0  # conservative planning bound; actual drain is randomized in orchestrator
MIN_DRAIN, MAX_DRAIN = 0.1, 10.0
MAX_ITERATIONS = 4
LAST_ERROR: Optional[str] = None
LAST_TRACE: List[Dict[str, str]] = []


class ParseTripArgs(BaseModel):
    drone_id: str
    destination: str
    drain_per_cell: float = Field(default=DEFAULT_DRAIN, ge=MIN_DRAIN, le=MAX_DRAIN)
    deadline: Optional[str] = None
    charge_to_pct: Optional[float] = Field(default=None, ge=0, le=100)


class FinishArgs(BaseModel):
    drone_id: str
    destination: str
    drain_per_cell: float = Field(ge=MIN_DRAIN, le=MAX_DRAIN)
    deadline: Optional[str] = None
    charge_to_pct: Optional[float] = Field(default=None, ge=0, le=100)


class MultiTripItem(BaseModel):
    drone_id: str
    destination: str
    deadline: Optional[str] = None


class ParseMultiArgs(BaseModel):
    trips: List[MultiTripItem]
    drain_per_cell: float = Field(default=DEFAULT_DRAIN, ge=MIN_DRAIN, le=MAX_DRAIN)


class FinishMultiArgs(BaseModel):
    trips: List[MultiTripItem]
    drain_per_cell: float = Field(ge=MIN_DRAIN, le=MAX_DRAIN)


class AgentStep(BaseModel):
    thought: str
    action: str
    action_input: Dict[str, Any]


TOOL_ARG_MODELS = {
    "parse_trip": ParseTripArgs,
    "finish": FinishArgs,
    "parse_multi": ParseMultiArgs,
    "finish_multi": FinishMultiArgs,
}

SYSTEM_PROMPT = """
You are the natural-language command parser for an autonomous Drone routing system.
Use a manual ReAct loop. Every response MUST be exactly one JSON object and nothing else:
{"thought":"...","action":"parse_trip|finish|parse_multi|finish_multi","action_input":{...}}

If the operator names ONE drone, use parse_trip first and then finish.
If the operator names TWO OR MORE drones, use parse_multi first and then finish_multi.
For multi-drone commands, action_input MUST be:
{"trips":[{"drone_id":"DRONE_01","destination":"AIRPORT|SUPERMARKET","deadline":"10:30 AM or null"},...],"drain_per_cell":2.0}
All drones in a multi-drone command share the SAME drain_per_cell value.
Never ask the operator to select drones or destinations through a form. Extract them from the natural-language command.
The known drone IDs are provided by the user message. Do not invent a drone ID that is not in that list.
Destination defaults to AIRPORT only when the operator does not specify one.
Battery drain is not fixed anymore. The physical model randomly consumes
1.5%-3.0% per normal movement cell. If the operator gives an old fixed drain
value, preserve it in the parsed JSON for compatibility, but the orchestrator
will use the 3.0% worst-case bound for planning and verification.
If the operator explicitly gives a deadline, preserve it.
Charging percentage is NOT an operator input; charging is calculated automatically by the routing system.
"""


def _normalise_id(raw) -> Optional[str]:
    m = re.search(r"\bdrone[-_ ]?0*(\d+)\b", str(raw).lower())
    return f"DRONE_{int(m.group(1)):02d}" if m else (str(raw) if raw else None)


def _extract_requested_drone_id(command: str) -> Optional[str]:
    m = re.search(r"\bdrone[-_ ]?0*(\d+)\b", command.lower())
    return f"DRONE_{int(m.group(1)):02d}" if m else None


def _normalise_destination(raw) -> str:
    text = str(raw or "").lower()
    if any(w in text for w in ("super", "market", "grocer", "shop", "store")):
        return "SUPERMARKET"
    return "AIRPORT"


def _extract_explicit_destination(command: str) -> Optional[str]:
    """Extract the destination explicitly named by the operator.

    The operator's destination is authoritative. This prevents an LLM response
    such as AIRPORT from overriding a command that explicitly says supermarket.
    """
    text = command.lower()
    if re.search(r"\bsupermarket\b|\bgrocery\b|\bgrocer\b", text):
        return "SUPERMARKET"
    if re.search(r"\bairport\b", text):
        return "AIRPORT"
    return None


def _extract_deadline(command: str) -> Optional[str]:
    patterns = [
        r"\bby\s+(\d{1,2}(?::\d{2})?\s*(?:am|pm)?)\b",
        r"\bdeadline\s+(?:is\s+)?(\d{1,2}(?::\d{2})?\s*(?:am|pm)?)\b",
    ]
    for pattern in patterns:
        m = re.search(pattern, command.lower())
        if m:
            return m.group(1).strip()
    return None


def _extract_explicit_drain(command: str) -> Optional[float]:
    """Extract the battery usage explicitly stated by the operator.

    If the command says 3% per cell/move, that value must override the
    LLM/default value of 2%.
    """
    patterns = [
        r"(\d+(?:\.\d+)?)\s*%?\s*(?:of\s+)?(?:battery\s+)?(?:per|/|a|each)\s+(?:grid\s+)?(?:cell|square|tile|step|move)",
        r"(?:uses?|uses\s+up|draws?|consumes?)\s+(\d+(?:\.\d+)?)\s*(?:%|percent)\s*(?:of\s+)?(?:battery)?\s*(?:per|for)\s+(?:cell|step|move)",
    ]
    text = command.lower()
    for pattern in patterns:
        m = re.search(pattern, text)
        if m:
            return max(MIN_DRAIN, min(MAX_DRAIN, float(m.group(1))))
    return None


def _extract_charge_to(command: str) -> Optional[float]:
    patterns = [
        r"charge\s+(?:the\s+drone\s+)?to\s+(\d+(?:\.\d+)?)\s*%",
        r"charge\s+(?:up\s+)?(?:until|till)\s+(\d+(?:\.\d+)?)\s*%",
    ]
    for pattern in patterns:
        m = re.search(pattern, command.lower())
        if m:
            return max(0.0, min(100.0, float(m.group(1))))
    return None


def _extract_all_drone_ids(command: str) -> List[str]:
    ids = []
    for m in re.finditer(r"\bdrone[-_ ]?0*(\d+)\b", command.lower()):
        did = f"DRONE_{int(m.group(1)):02d}"
        if did not in ids:
            ids.append(did)
    return ids


def _extract_multi_trips(command: str) -> List[Dict[str, Any]]:
    """Fallback parser for natural-language commands containing multiple drones."""
    text = command.lower()
    matches = list(re.finditer(r"\bdrone[-_ ]?0*(\d+)\b", text))
    if len(matches) < 2:
        return []

    trips = []
    for i, m in enumerate(matches):
        did = f"DRONE_{int(m.group(1)):02d}"
        segment_end = matches[i + 1].start() if i + 1 < len(matches) else len(text)
        segment = text[m.end():segment_end]
        destination = None
        if re.search(r"\bsupermarket\b|\bgrocery\b|\bgrocer\b|\bmarket\b", segment):
            destination = "SUPERMARKET"
        elif re.search(r"\bairport\b", segment):
            destination = "AIRPORT"
        if destination is not None:
            trips.append({"drone_id": did, "destination": destination, "deadline": _extract_deadline(segment)})

    # Commands such as "Drone-01 and Drone-02 to the airport" put the
    # destination after the final drone mention, so apply that destination to
    # any still-unassigned drone when exactly one destination is explicit.
    if len(trips) < len(matches):
        explicit = _extract_explicit_destination(text)
        if explicit:
            assigned = {t["drone_id"] for t in trips}
            for m in matches:
                did = f"DRONE_{int(m.group(1)):02d}"
                if did not in assigned:
                    trips.append({"drone_id": did, "destination": explicit, "deadline": _extract_deadline(text)})

    order = {f"DRONE_{int(m.group(1)):02d}": i for i, m in enumerate(matches)}
    trips.sort(key=lambda t: order.get(t["drone_id"], 999))
    return trips


def _fallback_parse(command: str) -> Dict[str, Any]:
    drone_id = _normalise_id(command) if re.search(r"\bdrone[-_ ]?\d", command.lower()) else None
    match = re.search(
        r"(\d+(?:\.\d+)?)\s*(?:%|percent)?\s*(?:of\s+)?(?:battery\s+)?"
        r"(?:per|/|a|each)\s+(?:grid\s+)?(?:cell|square|tile|step|move)",
        command.lower(),
    )
    if match is None:
        match = re.search(
            r"(?:uses?|uses\s+up|draws?|consumes?)\s+(\d+(?:\.\d+)?)\s*"
            r"(?:%|percent|kwh|kw/?h)?(?:\s+(?:of\s+)?battery)?(?:\s+(?:per|for)\s+(?:cell|step|move))?",
            command.lower(),
        )
    return {
        "drone_id": drone_id,
        "destination": _normalise_destination(command),
        "drain_per_cell": float(match.group(1)) if match else DEFAULT_DRAIN,
        "deadline": _extract_deadline(command),
        "charge_to_pct": _extract_charge_to(command),
    }


def _strip_fences(text: str) -> str:
    text = text.strip()
    text = re.sub(r"^```(?:json)?\s*", "", text, flags=re.I)
    text = re.sub(r"\s*```$", "", text)
    start, end = text.find("{"), text.rfind("}")
    return text[start:end + 1] if start >= 0 and end >= start else text


def _chat(messages: List[Dict[str, str]]) -> Optional[str]:
    global LAST_ERROR
    key = os.environ.get("OPENAI_API_KEY")
    if not key:
        LAST_ERROR = "no OPENAI_API_KEY set in the environment"
        return None
    try:
        from openai import OpenAI
        response = OpenAI(api_key=key).chat.completions.create(
            model=OPENAI_MODEL,
            response_format={"type": "json_object"},
            messages=messages,
        )
        LAST_ERROR = None
        return response.choices[0].message.content or ""
    except Exception as exc:
        LAST_ERROR = f"OpenAI call failed: {exc}"
        return None


def _parse_react(command: str, known_drone_ids: List[str]) -> Optional[Dict[str, Any]]:
    global LAST_TRACE
    LAST_TRACE = []
    messages = [{
        "role": "user",
        "content": f"Known drone IDs: {', '.join(known_drone_ids)}\nOperator command: {command}",
    }]

    for _ in range(MAX_ITERATIONS):
        raw = _chat([{"role": "system", "content": SYSTEM_PROMPT}] + messages)
        if raw is None:
            return None
        messages.append({"role": "assistant", "content": raw})
        LAST_TRACE.append({"role": "assistant", "content": raw})

        try:
            step_data = json.loads(_strip_fences(raw))
            step = AgentStep.model_validate(step_data)
        except (json.JSONDecodeError, ValidationError) as exc:
            observation = f"ERROR: invalid ReAct JSON/schema: {exc}"
            messages.append({"role": "user", "content": observation})
            LAST_TRACE.append({"role": "user", "content": observation})
            continue

        if step.action not in TOOL_ARG_MODELS:
            observation = f"ERROR: unknown action '{step.action}'. Valid actions: {list(TOOL_ARG_MODELS)}"
            messages.append({"role": "user", "content": observation})
            LAST_TRACE.append({"role": "user", "content": observation})
            continue

        try:
            validated = TOOL_ARG_MODELS[step.action].model_validate(step.action_input)
        except ValidationError as exc:
            observation = f"ERROR: invalid action_input: {exc}"
            messages.append({"role": "user", "content": observation})
            LAST_TRACE.append({"role": "user", "content": observation})
            continue

        if step.action in ("parse_trip", "parse_multi"):
            data = validated.model_dump()
            if step.action == "parse_trip":
                data["destination"] = _normalise_destination(data["destination"])
                data["drone_id"] = _normalise_id(data["drone_id"])
            else:
                for item in data.get("trips", []):
                    item["destination"] = _normalise_destination(item["destination"])
                    item["drone_id"] = _normalise_id(item["drone_id"])
            observation = f"OBSERVATION: parsed candidate {json.dumps(data)}"
            messages.append({"role": "user", "content": observation})
            LAST_TRACE.append({"role": "user", "content": observation})
            continue

        return validated.model_dump()

    return None


def parse_command(command: str, known_drone_ids) -> Dict[str, Any]:
    """Parse one or many drone commands from natural language. Never raises."""
    known_drone_ids = list(known_drone_ids)
    multi_hint = len(_extract_all_drone_ids(command)) >= 2

    result = _parse_react(command, known_drone_ids)
    used_llm = result is not None
    error = None if used_llm else LAST_ERROR

    if multi_hint:
        # The multi-drone schema is deliberately kept separate from the single
        # drone schema so the LLM cannot silently collapse several assignments
        # into one drone.
        if result is None or "trips" not in result:
            result = {
                "trips": _extract_multi_trips(command),
                "drain_per_cell": _extract_explicit_drain(command) or DEFAULT_DRAIN,
            }
            used_llm = False
        trips = []
        explicit_drain = _extract_explicit_drain(command)
        for item in result.get("trips", []):
            did = _normalise_id(item.get("drone_id"))
            dest = _normalise_destination(item.get("destination"))
            if did in known_drone_ids:
                trips.append({
                    "drone_id": did,
                    "destination": dest,
                    "deadline": _extract_deadline(command) or item.get("deadline"),
                })
        # If the LLM returned the wrong shape, fall back to deterministic
        # extraction from the actual operator sentence.
        if len(trips) != len(_extract_all_drone_ids(command)):
            fallback = _extract_multi_trips(command)
            trips = [t for t in fallback if t["drone_id"] in known_drone_ids]
        drain = explicit_drain
        if drain is None:
            try:
                drain = float(result.get("drain_per_cell", DEFAULT_DRAIN))
            except (TypeError, ValueError):
                drain = DEFAULT_DRAIN
        drain = max(MIN_DRAIN, min(MAX_DRAIN, drain))
        return {
            "multi": True,
            "trips": trips,
            "drain_per_cell": drain,
            "_source": "react_llm" if used_llm else "fallback_rules",
            "_llm_error": error,
        }

    if result is None:
        result = _fallback_parse(command)

    requested_id = _extract_requested_drone_id(command)
    did = requested_id or _normalise_id(result.get("drone_id"))
    if did not in known_drone_ids:
        result["_unresolved_drone_id"] = did or "<none named>"
        did = None
    result["drone_id"] = did
    explicit_destination = _extract_explicit_destination(command)
    result["destination"] = explicit_destination or _normalise_destination(result.get("destination"))
    explicit_deadline = _extract_deadline(command)
    result["deadline"] = explicit_deadline or result.get("deadline")
    explicit_drain = _extract_explicit_drain(command)
    if explicit_drain is not None:
        result["drain_per_cell"] = explicit_drain
    else:
        try:
            drain = float(result.get("drain_per_cell", DEFAULT_DRAIN))
        except (TypeError, ValueError):
            drain = DEFAULT_DRAIN
        result["drain_per_cell"] = max(MIN_DRAIN, min(MAX_DRAIN, drain))
    # Operator charging targets are intentionally ignored: charging is automatic.
    result["charge_to_pct"] = None
    result["_source"] = "react_llm" if used_llm else "fallback_rules"
    result["_llm_error"] = error
    return result


def test_llm_connection() -> Dict[str, Any]:
    key = os.environ.get("OPENAI_API_KEY")
    if not key:
        return {"ok": False, "reason": "no OPENAI_API_KEY set in the environment"}
    try:
        from openai import OpenAI
        response = OpenAI(api_key=key).chat.completions.create(
            model=OPENAI_MODEL,
            max_completion_tokens=200,
            messages=[{"role": "user", "content": "Reply with exactly one word: PONG"}],
        )
        return {"ok": True, "model": OPENAI_MODEL,
                "reply": (response.choices[0].message.content or "").strip()}
    except Exception as exc:
        return {"ok": False, "reason": f"OpenAI call failed: {exc}"}
