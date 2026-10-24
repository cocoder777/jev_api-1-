"""Start the official Kev service offline, using a localized released checkpoint."""
import argparse
import os
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[2]


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--host',default='127.0.0.1')
    p.add_argument('--port',type=int,default=18028)
    p.add_argument('--device',choices=['cpu','cuda'],default='cuda')
    a=p.parse_args()
    path=ROOT/'models/kev-4b-local'
    if not (path/'head.pt').is_file():raise SystemExit('Run server/local_models/setup_kev.py first')
    os.environ['HF_HUB_OFFLINE']='1';os.environ['TRANSFORMERS_OFFLINE']='1'
    os.environ.setdefault('KEV_PREFIX_CACHE','0')
    # Start with the official eager path; avoid hidden compilation on minimal hosts.
    os.environ.setdefault('KEV_CUDA_GRAPHS','0');os.environ.setdefault('KEV_FUSED','0')
    os.environ.setdefault('KEV_DTYPE','bf16' if a.device=='cuda' else 'fp32')
    os.environ.setdefault('OMP_NUM_THREADS','4')
    sys.argv=['kev.serve','--run',str(path),'--host',a.host,'--port',str(a.port),'--device',a.device]
    from kev.serve import main as serve
    serve()


if __name__=='__main__':main()
