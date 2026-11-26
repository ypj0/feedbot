#!/usr/bin/env python3
"""Run bounded maintenance executable against PTYs, never the robot."""
import errno, json, os, pathlib, pty, select, subprocess, sys, time, tty

STATIONARY=bytes.fromhex('7f7f280200000000000000000000000100010000000107141414aa15e001c70a0000000000210d0a')
def response(family, emergency=1, baud=115200):
    frame=bytearray(40)
    frame[:5]=bytes([127,127,40,family,96]);frame[38:]=b'\r\n'
    if family==1:frame[5:9]=baud.to_bytes(4,'big')
    else:
        frame[5]=emergency;frame[6]=1;frame[33]=2;frame[35:37]=(441).to_bytes(2,'big')
    frame[37]=(-sum(frame[:37]))&255
    return bytes(frame)

def run(binary,case):
    master,slave=pty.openpty();tty.setraw(slave);path=os.ttyname(slave)
    proc=subprocess.Popen([binary,'--pty',path],stdout=subprocess.PIPE,stderr=subprocess.PIPE)
    os.close(slave);pending=bytearray();runtime=0;begin=time.monotonic()
    try:
        while proc.poll() is None:
            if time.monotonic()-begin>3:raise AssertionError('probe timeout')
            if not select.select([master],[],[],.01)[0]:continue
            try:data=os.read(master,4096)
            except OSError as exc:
                if exc.errno==errno.EIO:
                    time.sleep(.001);continue
                raise
            pending.extend(data)
            while len(pending)>=40:
                f=bytes(pending[:40]);del pending[:40]
                assert f[:3]==b'\x7f\x7f\x28' and sum(f[:38])%256==0
                if f[3]==1:
                    if case!='silent_handshake':os.write(master,response(1,baud=9600 if case=='wrong_baud' else 115200))
                else:
                    assert f==STATIONARY,'unexpected action or nonzero velocity'
                    runtime+=1
                    if case!='silent_runtime':os.write(master,response(2,emergency=0 if case=='emergency_released' else 1))
        stdout,stderr=proc.communicate(timeout=1)
        assert (proc.returncode==0)==(case=='healthy'),(case,stdout,stderr)
        if case=='healthy':
            report=json.loads(stdout.decode().splitlines()[-1]);assert report['rx']>=15 and runtime>=15
        else:
            expected={'wrong_baud':'baud mismatch','silent_handshake':'handshake timeout',
                      'silent_runtime':'feedback stale','emergency_released':'maintenance feedback condition failed'}[case]
            assert expected in stderr.decode(),(case,stderr)
        print('PASS stationary probe',case,'runtime_frames',runtime)
    finally:
        if proc.poll() is None:proc.kill();proc.wait()
        os.close(master)

if __name__=='__main__':
    for case in ('healthy','wrong_baud','silent_handshake','silent_runtime','emergency_released'):run(sys.argv[1],case)
