"""Simple log parser for serial outputs
Parses lines like: ERR=...,U=...,L=...,R=... or wL=...,wR=...,vref=...
Usage: python log_parser.py log.txt
"""
import re, sys, json
from statistics import mean

RE_PROTO1 = re.compile(r"ERR=(?P<err>-?\d+),U=(?P<u>-?\d+\.?\d*),L=(?P<L>-?\d+),R=(?P<R>-?\d+)")
RE_PROTO2 = re.compile(r"wL=(?P<wL>-?\d+\.?\d*),wR=(?P<wR>-?\d+\.?\d*),vref=(?P<vref>-?\d+\.?\d*),I_L=(?P<I_L>-?\d+\.?\d*),I_R=(?P<I_R>-?\d+\.?\d*)")

def main(path):
    p1, p2 = [], []
    for line in open(path,'r',encoding='utf-8',errors='ignore'):
        m1 = RE_PROTO1.search(line)
        if m1:
            p1.append({k:float(v) for k,v in m1.groupdict().items()})
            continue
        m2 = RE_PROTO2.search(line)
        if m2:
            p2.append({k:float(v) for k,v in m2.groupdict().items()})
    summary = {}
    if p1:
        summary['proto1'] = {
            'samples': len(p1),
            'mean_abs_err': mean(abs(d['err']) for d in p1),
            'mean_command': mean(d['u'] for d in p1)
        }
    if p2:
        summary['proto2'] = {
            'samples': len(p2),
            'mean_wL': mean(d['wL'] for d in p2),
            'mean_wR': mean(d['wR'] for d in p2)
        }
    print(json.dumps(summary, indent=2))

if __name__ == '__main__':
    if len(sys.argv)<2:
        print("Usage: python log_parser.py serial.log")
        raise SystemExit(1)
    main(sys.argv[1])
