"""Pinned Kev runtime/model setup in the dedicated .venv-kev interpreter."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[2]
RUNTIME_REVISION='1f04cfa28e6a708485b28a9585010b9e436b776e'
ADAPTER_REVISION='6cfce5c2fa4b4bd64026336ab649c5ca78857d52'
BASE_REVISION='1001bb4d826a52d1f399e183466143f4da7b741b'


def digest(p):
    with p.open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()


def prepare():
    import torch
    adapter=ROOT/'models/kev-4b';base=ROOT/'models/kev-4b-base';target=ROOT/'models/kev-4b-local'
    if not (base/'config.json').is_file() or not list(base.glob('*.safetensors')):
        raise RuntimeError('Kev base weights missing; complete the pinned model download first')
    meta=torch.load(adapter/'head.pt',map_location='cpu',weights_only=True)
    if meta.get('base_revision') != BASE_REVISION:
        raise RuntimeError('Released head expects a different base revision')
    target.mkdir(parents=True,exist_ok=True)
    # Only deployment path metadata changes. Released files stay intact.
    for name in ('adapter_config.json','adapter_model.safetensors'):
        dest=target/name
        if not dest.exists():dest.symlink_to(os.path.relpath(adapter/name,target))
    meta['base']=str(base.resolve());meta['base_revision']=None
    torch.save(meta,target/'head.pt')
    manifest=dict(runtime_revision=RUNTIME_REVISION,adapter_repo='jaredpalmer/kev-4b',
        adapter_revision=ADAPTER_REVISION,base_repo='Qwen/Qwen3.5-4B-Base',base_revision=BASE_REVISION,
        original_head_sha256=digest(adapter/'head.pt'),adapter_sha256=digest(adapter/'adapter_model.safetensors'),
        localized_head_sha256=digest(target/'head.pt'),localized_base=str(base.resolve()))
    (target/'deployment.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(f'Offline deployment prepared: {target}')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--prepare-only',action='store_true')
    p.add_argument('--skip-install',action='store_true')
    a=p.parse_args();runtime=ROOT/'models/kev-runtime'
    if not a.prepare_only:
        if not runtime.exists():
            subprocess.run(['git','clone','https://github.com/jaredpalmer/kev.git',str(runtime)],check=True)
            subprocess.run(['git','-C',str(runtime),'checkout',RUNTIME_REVISION],check=True)
        revision=subprocess.check_output(['git','-C',str(runtime),'rev-parse','HEAD'],text=True).strip()
        if revision!=RUNTIME_REVISION:raise RuntimeError('Kev runtime revision differs from this deployment')
        if not a.skip_install:
            subprocess.run([sys.executable,'-m','pip','install','-e',f'{runtime}[serve]'],check=True)
        from huggingface_hub import snapshot_download
        snapshot_download('jaredpalmer/kev-4b',revision=ADAPTER_REVISION,local_dir=str(ROOT/'models/kev-4b'),max_workers=4)
        snapshot_download('Qwen/Qwen3.5-4B-Base',revision=BASE_REVISION,local_dir=str(ROOT/'models/kev-4b-base'),max_workers=4,
                          ignore_patterns=['*.md','*.png','*.jpg','*.gguf'])
    prepare()


if __name__=='__main__':main()
