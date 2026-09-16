"""Fetch only the pinned, Apache-2.0 SO101 model; never fetch policy weights."""

from __future__ import annotations

import hashlib
import json
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from urllib.request import urlopen

REVISION = "ac6b2b09983786f3036cab1000221017fa2193b4"
BASE = "https://raw.githubusercontent.com/google-deepmind/mujoco_menagerie"
DEST = Path(__file__).resolve().parents[1] / "src/lerobot_env_so101/assets/so101"


def fetch(relative: str) -> tuple[str, str]:
    data = urlopen(f"{BASE}/{REVISION}/robotstudio_so101/{relative}", timeout=60).read()
    target = DEST / relative
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_bytes(data)
    return relative, hashlib.sha256(data).hexdigest()


def main() -> None:
    url = f"https://api.github.com/repos/google-deepmind/mujoco_menagerie/git/trees/{REVISION}?recursive=1"
    tree = json.loads(urlopen(url, timeout=60).read())["tree"]
    prefix = "robotstudio_so101/"
    paths = [
        item["path"][len(prefix) :]
        for item in tree
        if item["type"] == "blob" and item["path"].startswith(prefix) and not item["path"].endswith(".png")
    ]
    with ThreadPoolExecutor(max_workers=4) as pool:
        hashes = dict(pool.map(fetch, paths))
    manifest = {
        "repository": "google-deepmind/mujoco_menagerie",
        "revision": REVISION,
        "license": "Apache-2.0",
        "sha256": hashes,
    }
    (DEST / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(f"Fetched {len(hashes)} files to {DEST}")


if __name__ == "__main__":
    main()
