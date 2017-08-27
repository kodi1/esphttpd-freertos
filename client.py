# -*- coding: utf-8 -*-
import socket   #for sockets
import sys  #for exit
import time
import struct
import binascii
import json as json
import random
droid = None

try:
    import imaplib
    import android
    droid=android.Android()
    print 'Run on android'
except:
    pass

cnt = 0
port = 6666
ip = None
len_msg = 128

def calc_sum(packet):
    xor = 0
    i = 0
    while i < len(packet):
        xor = xor ^ ord(packet[i])
        i += 1
    return xor

def discover():
    ret = []
    s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    req = {'payload': {'cmd_id': 1, 'cnt': cnt}}
    req['sum'] =  calc_sum(json.dumps(req['payload'], separators=(',', ':')))
    s.sendto(json.dumps(req), ('<broadcast>', port))

    while True:
        try:
            data, server = s.recvfrom(len_msg)
        except socket.timeout:
            if not len(ret):
                print "Timeout"
                ret = None
            break
        else:
            d = json.loads(data)
            if d['sum'] == calc_sum(json.dumps(d['payload'], separators=(',', ':'))):
                ret.append(d['payload'])
            else:
                print 'wrong data'

    s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 0)
    return ret

def send_cmd(js):
    req = {'payload': js}
    req['sum'] = calc_sum(json.dumps(req['payload'], separators=(',', ':')))
    s.sendto(json.dumps(req), (ip, port))
    sys.stdout.write('\rcnt: %d' % js['cnt'])
    sys.stdout.flush()
    try:
        data, server = s.recvfrom(len_msg)
    except socket.timeout:
        print 'Timeout'
        global cnt
        cnt = 0
        #sys.exit()
        pass
    else:
        d = json.loads(data)
        if d['sum'] == calc_sum(json.dumps(d['payload'], separators=(',', ':'))):
            return d['payload']
        else:
            print 'wrong data'

def user_in():
    js = {}
    in_put = raw_input('1 - r g b\n2 - pwm period\n: ')

    if len(in_put):
        try:
            i = [int(i) for i in in_put.split(' ')]
        except:
            return js

        if len(i) == 4:
            if i[0] == 1:
                js['cmd_id'] = 5
                js['led'] = {'r': i[1], 'g': i[2], 'b': i[3]}
                js['cnt'] = cnt
        elif len(i) == 2:
            if i[0] == 2:
                js['cmd_id'] = 3
                js['period'] = i[1]
                js['cnt'] = cnt
    return js

def gen1 (l):
    for k in l.keys():
        if l[k]['step'] > 0:
            for val in range(l[k]['min'], l[k]['max'], l[k]['step']):
                l[k]['current'] = val
                time.sleep(l[k]['t'])
                yield l['r']['current'], l['g']['current'], l['b']['current']
            l[k]['step'] *= -1
        else:
            for val in range(l[k]['max'], l[k]['min'], l[k]['step']):
                l[k]['current'] = val
                time.sleep(l[k]['t'])
                yield l['r']['current'], l['g']['current'], l['b']['current']
            l[k]['step'] *= -1

if __name__ == '__main__':
    if len(sys.argv) == 2:
        t = int(sys.argv[1])
        r = g = b = None
    elif len(sys.argv) == 5:
        t = float(sys.argv[1])
        r = int(sys.argv[2])
        g = int(sys.argv[3])
        b = int(sys.argv[4])
        print 'Timeout %f, R: %d, G: %d, B: %d' % (t, r, g, b)
    elif droid:
        t = float(droid.getIntent().result[u'extras'][u'%timeout'])
        r = int(droid.getIntent().result[u'extras'][u'%red'])
        g = int(droid.getIntent().result[u'extras'][u'%green'])
        b = int(droid.getIntent().result[u'extras'][u'%blue'])
        print 'Timeout %f, R: %d, G: %d, B: %d' % (t, r, g, b)
    else:
        print 'wrong arguments'
        sys.exit()
    cnt += 1
    js = {'cnt': cnt}

    # create dgram udp socket
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    except socket.error:
        print 'Failed to create socket'
        sys.exit()
    else:
        s.settimeout(t)

    if ip is None:
        while True:
            print 'Try to find Led device %d' % cnt
            _discover = discover()
            for _d in _discover:
                ip = '%d.%d.%d.%d' % (_d['ip'][0], _d['ip'][1], _d['ip'][2], _d['ip'][3],)
                print 'Found devices ip: %s mac: %s' % (ip, str(_d['mac']))
            if _discover:
                break

            if cnt < 10:
                cnt += 1
                continue
            else:
                print 'Led device not found'
                sys.exit()

    js['cmd_id'] = 2
    try:
        print send_cmd(js)
    except:
        pass
    js['cmd_id'] = 4
    print send_cmd(js)

    if r is not None and g is not None and b is not None:
        js['cmd_id'] = 5
        js['led'] = {'r': r, 'g': g, 'b': b}
        js['cnt'] = cnt
        print send_cmd(js)
        cnt += 1
    else:
        while(1) :
            js = user_in()
            if js:
                print send_cmd(js)
            else:
                led = {
                        'r' : {'min': 0, 'max': 900, 'current': 0, 'step': 10, 't': 0.01},
                        'g' : {'min': 0, 'max': 900, 'current': 0, 'step': 10, 't': 0.01},
                        'b' : {'min': 0, 'max': 900, 'current': 0, 'step': 10, 't': 0.01},
                        }
                while True:
                    #r = random.randint(led['r']['min'], led['r']['max'])
                    #b = random.randint(led['b']['min'], led['b']['max'])
                    #g = random.randint(led['g']['min'], led['g']['max'])
                    #js['cmd_id'] = 5
                    #js['led'] = {'r': r, 'g': g, 'b': b}
                    #js['cnt'] = cnt
                    #send_cmd(js)
                    #cnt += 1
                    for r, g, b in gen1(led):
                        js['cmd_id'] = 5
                        js['led'] = {'r': r, 'g': g, 'b': b}
                        js['cnt'] = cnt
                        send_cmd(js)
                        cnt += 1
