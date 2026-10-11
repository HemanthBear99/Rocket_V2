"""Offline license keys (Ed25519-signed) with a 14-day evaluation period.

A key is ``<base64url(json payload)>.<base64url(signature)>``; the payload
holds the licensee, plan and expiry date. Only the public key ships with the
app, so keys cannot be minted from the exe. Keys are issued with
tools/issue_license.py and the private key kept outside the repository.

Enforcement applies to the packaged desktop app only (sys.frozen) unless
RLV_ENFORCE_LICENSE=1, so running from source and the test suite are
unaffected.
"""

from __future__ import annotations

import base64
import json
import os
import sys
from datetime import date, datetime, timedelta, timezone
from typing import Any

from .app_paths import data_dir

PUBLIC_KEY_B64 = "IhdUKdYvq/SaHP5wxy3Gd26io2shGIrEzP5Ms1uwdWs="
TRIAL_DAYS = 14


def _b64d(text: str) -> bytes:
    return base64.urlsafe_b64decode(text + "=" * (-len(text) % 4))


def verify_key(key: str) -> dict[str, Any]:
    """Payload of a valid key; raises ValueError otherwise."""
    from cryptography.exceptions import InvalidSignature
    from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PublicKey

    try:
        body, sig = key.strip().split(".")
        payload_raw = _b64d(body)
        Ed25519PublicKey.from_public_bytes(base64.b64decode(PUBLIC_KEY_B64)).verify(_b64d(sig), payload_raw)
        payload = json.loads(payload_raw)
    except (ValueError, InvalidSignature, json.JSONDecodeError) as exc:
        raise ValueError("License key is not valid") from exc
    if date.fromisoformat(payload["expires"]) < date.today():
        raise ValueError(f"License expired on {payload['expires']}")
    return payload


def enforced() -> bool:
    return bool(getattr(sys, "frozen", False)) or os.environ.get("RLV_ENFORCE_LICENSE") == "1"


def _trial_start() -> datetime:
    marker = data_dir() / "trial_started"
    if not marker.exists():
        marker.write_text(datetime.now(timezone.utc).isoformat(), encoding="utf-8")
    return datetime.fromisoformat(marker.read_text(encoding="utf-8").strip())


def status() -> dict[str, Any]:
    key_file = data_dir() / "license.key"
    invalid = None
    if key_file.exists():
        try:
            p = verify_key(key_file.read_text(encoding="utf-8"))
        except ValueError as exc:
            invalid = str(exc)
        else:
            return {"state": "licensed", "licensee": p.get("name"), "plan": p.get("plan"),
                    "expires": p["expires"], "can_run": True, "enforced": enforced()}
    days_left = TRIAL_DAYS - (datetime.now(timezone.utc) - _trial_start()).days
    can_run = days_left > 0 or not enforced()
    return {"state": "trial" if days_left > 0 else "expired", "trial_days_left": max(days_left, 0),
            "can_run": can_run, "enforced": enforced(), "problem": invalid}


def activate(key: str) -> dict[str, Any]:
    verify_key(key)
    (data_dir() / "license.key").write_text(key.strip(), encoding="utf-8")
    return status()


def require_license() -> None:
    """Raise PermissionError when runs are not allowed."""
    st = status()
    if not st["can_run"]:
        raise PermissionError("Evaluation period has ended: enter a license key in Settings -> About & License.")


def issue_key(private_key_pem: bytes, name: str, plan: str, days: int) -> str:
    """Sign a license (used by tools/issue_license.py)."""
    from cryptography.hazmat.primitives.serialization import load_pem_private_key

    payload = json.dumps({"name": name, "plan": plan,
                          "expires": (date.today() + timedelta(days=days)).isoformat()},
                         separators=(",", ":")).encode()
    sig = load_pem_private_key(private_key_pem, password=None).sign(payload)
    enc = lambda b: base64.urlsafe_b64encode(b).decode().rstrip("=")
    return f"{enc(payload)}.{enc(sig)}"
