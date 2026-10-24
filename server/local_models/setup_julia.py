"""Download a pinned complete Julia snapshot; install it into this interpreter's env."""
import argparse
import json
import hashlib
import subprocess
import sys
from pathlib import Path

REPO = 'SupersonicLabs/Julia-1'
REVISION = 'a85b127321d580d65176c89ced8273f305745d85'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--path', type=Path, default=Path(__file__).resolve().parents[2] / 'models/Julia-1')
    args = parser.parse_args()
    from huggingface_hub import snapshot_download
    snapshot_download(REPO, revision=REVISION, local_dir=str(args.path), max_workers=4)
    with (args.path / 'model.safetensors').open('rb') as weights:
        actual = hashlib.file_digest(weights, 'sha256').hexdigest()
    expected = 'df853bf7fe424420011f3d0c47a05d7341aa9eefa7fb9f203ea4aada4ad95b72'
    if actual != expected:
        raise RuntimeError('Julia checkpoint SHA-256 mismatch')
    subprocess.run([sys.executable, '-m', 'pip', 'install', '-e', str(args.path)], check=True)
    manifest = dict(repo=REPO, revision=REVISION, weights_sha256=actual)
    (args.path / 'deployment.json').write_text(json.dumps(manifest, indent=2) + '\n')
    print(f'Installed {REPO}@{REVISION} at {args.path}')


if __name__ == '__main__':
    main()
