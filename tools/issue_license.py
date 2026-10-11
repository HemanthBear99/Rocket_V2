"""Issue a Vyom license key.

    python -m rlv_sim.tools.issue_license "Customer Name" --plan pro --days 365 \
        --key ~/.boostback_signing/license_private.pem

The private key must never be committed or shipped; only licensing.py's
PUBLIC_KEY_B64 goes into the app.
"""

import argparse
from pathlib import Path

from rlv_sim.licensing import issue_key, verify_key


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("name")
    ap.add_argument("--plan", default="pro")
    ap.add_argument("--days", type=int, default=365)
    ap.add_argument("--key", default=str(Path.home() / ".boostback_signing" / "license_private.pem"))
    args = ap.parse_args()
    key = issue_key(Path(args.key).expanduser().read_bytes(), args.name, args.plan, args.days)
    verify_key(key)  # sanity: matches the app's public key
    print(key)


if __name__ == "__main__":
    main()
