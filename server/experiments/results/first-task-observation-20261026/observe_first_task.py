"""Receive-only observation; never binds the task port or sends controls."""
import json
import os
import selectors
import socket
import struct
import subprocess
import sys
import threading
import time
from collections import Counter, defaultdict
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path('/home/epic/jev_api(1)')
sys.path.insert(0, str(ROOT))
from server.decision_service import (parse_header, decode_situation_packet,
    analyze_task, FragmentReassembler)

OUT = ROOT / 'server/experiments/results/first-task-observation-20261026'
START = time.monotonic()
lock = threading.RLock()
done = threading.Event()
latest = {}
counts = Counter()
last_seen = {}
intervals = defaultdict(list)
position_ranges = {}
first_task = None
terminal = None
terminal_at = None
capture_error = None
network_count = 0
reassemblers = {}
events = (OUT / 'events.jsonl').open('a', buffering=1)


def emit(kind, **fields):
    row = dict(kind=kind, received_at=datetime.now(timezone.utc).isoformat(),
               elapsed_s=round(time.monotonic()-START, 6), **fields)
    with lock:
        events.write(json.dumps(row, ensure_ascii=False, allow_nan=False)+'\n')
    return row


def handle(payload, source, destination, channel):
    global first_task, terminal, terminal_at, network_count
    row = dict(channel=channel, source=list(source), destination=list(destination),
               length=len(payload), raw_hex=payload.hex(' '))
    try:
        header = parse_header(payload)
        row['header'] = header
        sid = header['secondaryInfoUnitId']
        if sid == 0x0060:
            state = decode_situation_packet(payload)
            row['state'] = state
            if channel == 'multicast':
                now = time.monotonic()
                pid = state['platformId']
                with lock:
                    counts[pid] += 1
                    if pid in last_seen:
                        intervals[pid].append(now-last_seen[pid])
                    last_seen[pid] = now
                    latest[pid] = state
                    extent = position_ranges.setdefault(pid, [state['longitude'],state['longitude'],state['latitude'],state['latitude']])
                    extent[0]=min(extent[0],state['longitude']);extent[1]=max(extent[1],state['longitude'])
                    extent[2]=min(extent[2],state['latitude']);extent[3]=max(extent[3],state['latitude'])
        elif sid in (0x0660,0x0664,0x0666):
            flow = tuple(source)+tuple(destination)
            decoder = reassemblers.setdefault(flow, FragmentReassembler())
            complete = decoder.add(payload, source)
            if complete is not None:
                body = analyze_task(complete)
                row['body'] = body
                with lock:
                    if sid == 0x0660 and first_task is None and body.get('code') == 'trackingMode':
                        first_task = body
                        (OUT/'first-task.json').write_text(json.dumps(body,ensure_ascii=False,indent=2)+'\n')
                        print('FIRST_TASK',body.get('id'),'waypoints',body.get('params',{}).get('planWaypointNum'),flush=True)
                    if sid == 0x0664 and first_task is not None and terminal is None and (body.get('simId'),body.get('id')) == (first_task.get('simId'),first_task.get('id')) and body.get('status') in (2,3):
                        terminal = body
                        terminal_at = time.monotonic()
                        print('FIRST_TASK_TERMINAL',json.dumps(body,ensure_ascii=False),flush=True)
        elif sid == 0x0662 and len(payload) == 20:
            pid, speed, course = struct.unpack('>IHH', payload[12:])
            row['control'] = dict(platformId=pid,speed_knots=speed*.01,course_deg=course*.01)
    except Exception as exc:
        row['decode_error'] = str(exc)
    if channel == 'pcap':
        with lock:
            network_count += 1
    emit('packet', **row)


def udp_from_frame(frame, linktype):
    if linktype == 276:
        if len(frame)<20 or struct.unpack('>H',frame[:2])[0] != 0x0800:return
        ip=frame[20:]
    elif linktype == 113:
        if len(frame)<16 or struct.unpack('>H',frame[14:16])[0] != 0x0800:return
        ip=frame[16:]
    elif linktype == 1:
        if len(frame)<14:return
        offset=14;proto=struct.unpack('>H',frame[12:14])[0]
        if proto in (0x8100,0x88a8):
            if len(frame)<18:return
            proto=struct.unpack('>H',frame[16:18])[0];offset=18
        if proto!=0x0800:return
        ip=frame[offset:]
    elif linktype in (101,228):ip=frame
    else:return
    if len(ip)<20 or ip[0]>>4 != 4 or ip[9] != 17:return
    if struct.unpack('>H',ip[6:8])[0] & 0x1fff:return
    ihl=(ip[0]&15)*4
    if len(ip)<ihl+8:return
    sport,dport,length,_=struct.unpack('>HHHH',ip[ihl:ihl+8])
    if not ({sport,dport}&{18010,18014,18123}):return
    return ip[ihl+8:ihl+length],(socket.inet_ntoa(ip[12:16]),sport),(socket.inet_ntoa(ip[16:20]),dport)


def exact(stream,n):
    data=b''
    while len(data)<n:
        part=stream.read(n-len(data))
        if not part:raise EOFError('capture stream ended')
        data+=part
    return data


def capture_loop(proc):
    global capture_error
    try:
        with (OUT/'network.pcap').open('wb') as raw:
            head=exact(proc.stdout,24);raw.write(head);raw.flush()
            endian='<' if head[:4] in (b'\xd4\xc3\xb2\xa1',b'\x4d\x3c\xb2\xa1') else '>'
            linktype=struct.unpack(endian+'I',head[20:24])[0]
            emit('pcap_ready',linktype=linktype)
            print('NETWORK_CAPTURE_READY linktype',linktype,flush=True)
            while not done.is_set():
                header=exact(proc.stdout,16)
                _,_,size,_=struct.unpack(endian+'IIII',header)
                if size>1024*1024:raise ValueError('invalid capture size')
                frame=exact(proc.stdout,size)
                raw.write(header+frame);raw.flush()
                decoded=udp_from_frame(frame,linktype)
                if decoded:handle(*decoded,'pcap')
    except Exception as exc:
        if not done.is_set():
            capture_error=str(exc)
            emit('capture_error',error=capture_error)
            print('NETWORK_CAPTURE_ERROR',capture_error,flush=True)


def snapshot():
    with lock:
        now=time.monotonic()
        state=dict(started_at_utc=datetime.fromtimestamp(time.time()-(now-START),timezone.utc).isoformat(),
            duration_s=round(now-START,2),first_task=first_task,terminal_status=terminal,
            network_packets=network_count,network_capture_error=capture_error,
            counts_by_platform=dict(counts),latest_records=dict(latest),
            age_s={k:round(now-v,3) for k,v in last_seen.items()},
            position_ranges=position_ranges,
            intervals_s={k:dict(count=len(v),min=round(min(v),4),max=round(max(v),4),mean=round(sum(v)/len(v),4)) for k,v in intervals.items() if v})
        temp=OUT/'summary.tmp'
        temp.write_text(json.dumps(state,ensure_ascii=False,indent=2)+'\n')
        temp.replace(OUT/'summary.json')
    return state


sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM,socket.IPPROTO_UDP)
sock.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
sock.bind(('',18010))
sock.setsockopt(socket.IPPROTO_IP,socket.IP_ADD_MEMBERSHIP,socket.inet_aton('239.255.0.10')+socket.inet_aton('192.168.3.202'))
sock.settimeout(.5)
errlog=(OUT/'tcpdump.stderr.log').open('a',buffering=1)
proc=subprocess.Popen(['sudo','-n','tcpdump','-i','any','-n','-s','0','-U','-w','-',
    'udp and (port 18010 or port 18014 or port 18123)'],stdout=subprocess.PIPE,stderr=errlog)
thread=threading.Thread(target=capture_loop,args=(proc,),daemon=True);thread.start()
print('MULTICAST_READY',str(OUT),flush=True)
emit('observer_started',pid=os.getpid(),group='239.255.0.10',port=18010,interface='192.168.3.202')
last_report=0.
try:
    while True:
        try:
            payload,remote=sock.recvfrom(65535)
            handle(payload,remote,('239.255.0.10',18010),'multicast')
        except socket.timeout:pass
        now=time.monotonic()
        if now-last_report>=5:
            state=snapshot();last_report=now
            print('HEARTBEAT',json.dumps(dict(elapsed_s=state['duration_s'],platforms=sorted(counts),
                  multicast_packets=sum(counts.values()),network_packets=network_count,
                  task_id=first_task.get('id') if first_task else None,
                  terminal_status=terminal.get('status') if terminal else None),ensure_ascii=False),flush=True)
        if terminal_at is not None and now-terminal_at>=10:
            emit('observer_finished',reason='first task terminal status plus 10 seconds')
            break
except KeyboardInterrupt:
    emit('observer_interrupted')
finally:
    done.set();sock.close()
    try:proc.terminate()
    except OSError:pass
    snapshot();events.close()
