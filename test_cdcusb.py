import usb.core
import usb.util
import usb.control
import time
import json
import random
import os,sys
import array

class kbexitException(Exception): pass

if os.name == 'nt':
    import msvcrt
    kbhit = msvcrt.kbhit
    getch = msvcrt.getch

REQ_SET= 0b01000001
REQ_GET= 0b11000001
ACM_CTRL_DTR =  0x01
ACM_CTRL_RTS =  0x02
ep_in_addr  = 0x83
ep_out_addr = 0x03

emptyarray = array.array('B', [0x00] * 64 ) 
RUN = True
kbqueue = []

def init():
    global dev
    # Look for a specific device and open it
    #
    dev = usb.core.find(idVendor=0x1A86, idProduct=0x8010) # Arduino Leonardo
    if dev is None: raise ValueError('Device not found')

    # Detach interfaces if Linux already attached a driver on it.
    #
    dev.set_configuration( )

    # set line state
    ret = dev.ctrl_transfer( REQ_SET , 0x22 , ACM_CTRL_DTR | ACM_CTRL_RTS , 0x00 , None )
    if ret<0: raise Exception("set line state failed")

    # set baudrate
    # 0x0008CA00 = 576000 0x00, 0xCA, 0x08, 0x00
    # 0x0001C200 = 115200 0x00, 0xC2, 0x01, 0x00
    # baudrates = 576000
    baudrates = 115200
    ret = dev.ctrl_transfer( REQ_SET , 0x20 , 0x00 , 0x00 
                            # , array.array('B', [0x00, 0xC2, 0x01, 0x00, 0x00, 0x00, 0x08]) 
                            # , array.array('B', [0x00, 0xCA, 0x08, 0x00, 0x00, 0x00, 0x08]) 
                            , array.array('B', baudrates.to_bytes(4,"little") + bytearray([0x00, 0x00, 0x08]) ) 
                            )
                            
    if ret<0: raise Exception("set line state failed")

def writeline( s ):
    global dev
    ret = dev.write( ep_out_addr , bytes(s,"ascii") + b"\r\n"  , 100 )
    if ret<0: raise Exception("write failed")

def readline():
    global dev, emptyarray, RUN, kbqueue
    rets = b""
    for i in range(50):
        try:
            if kbhit():
                ch = getch()
                if ch==b'\x1b': raise kbexitException()
                else: kbqueue.append( ch )
            ret = dev.read( ep_in_addr , 64 , 100 )
        except usb.core.USBTimeoutError: 
            continue
        except kbexitException:
            RUN = False
            break
        except usb.core.USBError as e:
            print( "USBError" , e )
            ret = emptyarray
            break
    
        if ret != emptyarray:
            # print( "data" , ret.tostring() )
            rets += ret.tobytes()
            if rets.endswith(b"\r\n"): break

    return rets.decode("ascii")

if True:
    init()
    while True:
        if kbhit() and getch()==b'\x1b': break
        print( readline() )
    sys.exit(0)

if False:
    msleep = lambda ms: time.sleep(ms/1000)
    j = {"r": 255,"g": 0 , "b": 0 }
    writeline( json.dumps( j ) )
    print( readline() )

    msleep(500)
    j = {"r": 0,"g": 255 , "b": 0 }
    writeline( json.dumps( j ) )
    print( readline() )

    msleep(500)
    j = {"r": 0,"g": 0 , "b": 255 }
    writeline( json.dumps( j ) )
    print( readline() )
    sys.exit(0)

if True:
    tick = lambda : int( time.time() * 1000 )
    msleep = lambda ms: time.sleep(ms/1000)
    n = 0
    t = 0
    mmax = 400
    maxt = mmax
    rgb = 0x000000
    m=0
    # maxp = 250
    maxp = 150
    T_STEP = tick()
    T_PRINT = tick()
    init()
    while RUN:
        if kbhit():
            ch = getch()
            if ch==b'\x1b': 
                RUN = False
                break
            else:
                kbqueue.append( ch )
                if ch==b" ": print("\n")
                else: print( "*** CH:", ch )

        while len(kbqueue)>0:
            ch = kbqueue.pop(0)
            if ch==b' ': 
                if maxt == mmax: maxt = 0
                elif maxt == 0: maxt = mmax

            
        if tick() - T_STEP < 60: continue
        T_STEP = tick()
        if maxt==0:
            p = 0
            rgb = 0x0000AA
        elif t>maxt: 
            p=0
            rgb = 0x0000FF
        elif t>maxt-20: 
            p=int( maxp * 0.40 )
            rgb=0x00FF00
        elif t>maxt-40: 
            p=int( maxp * 0.70 )
            rgb=0xAA0000
        elif t<=maxt-40: 
            p=maxp
            rgb=0xFF0000
        
        if t<maxt-80: 
            m = 2
        else:
            m = 0

        rgbbytes = rgb.to_bytes(3,"big")
        j = {"p": p , "r" : rgbbytes[0] , "g" : rgbbytes[1] , "b" : rgbbytes[2] , "m" : m}
        s = json.dumps( j )
        writeline( s )
        
        rets = readline()
        try:
            r = json.loads( rets )
        except json.JSONDecodeError as e:
            print( "JSONDecodeError" , rets )
            continue
        t = r.get("t",-1)
        n += 1
        if True or  n%5 == 0:
            # print( "p:", r.get("p") ,"t:", r.get("t",-1) ,"v:", r.get("v",-1) , "e:" , r.get("e",-1)  , "d:", r.get("d",-1) , n % 100, tick()-T_PRINT , "   ", end="\r" )
            print( 
                "p:", r.get("p") ,"t:", r.get("t",-1) ,"v:", r.get("v",-1) , "e:" , r.get("e",-1)  , "d:", r.get("d",-1), "m:", r.get("m",-1) , n % 100, tick()-T_PRINT , "        " 
                , end="\r"
                )
            T_PRINT = tick()





sys.exit(0)
if False:
    msleep = lambda ms: time.sleep(ms/1000)
    b = 0x0008CA00
    print( b.to_bytes(4,"little") )
    print( bytes( [0x00, 0xCA, 0x08, 0x00 ] ) )
    print( array.array('B', b.to_bytes(4,"little") + bytearray([0x00, 0x00, 0x08]) ) )
    print( array.array('B', [0x00, 0xCA, 0x08, 0x00, 0x00, 0x00, 0x08]) )
    sys.exit(0)


# cfg = dev.get_active_configuration()    
# print( cfg[(2,0)]  )
# print( cfg.interfaces() )

# print( cfg[(0,0)] )
# sys.exit(0)
# intf = cfg[(2,0)]

# print( intf )
# usb.util.claim_interface(dev, 0 )
# usb.util.claim_interface(dev, 1 )

if False:
    endp = intf.endpoints()
    epin, epout = None, None
    if usb.util.endpoint_direction( endp[0].bEndpointAddress ) == usb.util.ENDPOINT_IN:
        epin = endp[0]
    if usb.util.endpoint_direction( endp[1].bEndpointAddress ) == usb.util.ENDPOINT_OUT:
        epout = endp[1]
    # print( cfg )
    # print( "**IN" , epin )
    # print( "**OUT" , epout )
    if epin is None or epout is None: raise ValueError("Invalid endpoints")



if False:
    for itf_num in [0, 1]:
        itf = usb.util.find_descriptor(dev.get_active_configuration(),
                                    bInterfaceNumber=itf_num)
        # if dev.is_kernel_driver_active(itf):
        #     dev.detach_kernel_driver(itf)
        usb.util.claim_interface(dev, itf)


# set control line state 0x2221
"""
 Start configuring the device:
     * - set line state
     */
    #define ACM_CTRL_DTR   0x01
    #define ACM_CTRL_RTS   0x02

    rc = libusb_control_transfer(devh, 0x21, 0x22, ACM_CTRL_DTR | ACM_CTRL_RTS,
                                0, NULL, 0, 0);
"""

# dev.ctrl_transfer(0x21, 0x22, 0x01 | 0x02, 0, None)
ret = dev.ctrl_transfer( REQ_SET , 0x22 , 0x01 | 0x02 , 0x00 , None )
if ret<0: raise Exception("set line state failed")
# 0x0008CA00 = 576000 0x00, 0xCA, 0x08, 0x00
# 0x0001C200 = 115200 0x00, 0xC2, 0x01, 0x00
ret = dev.ctrl_transfer( REQ_SET , 0x20 , 0x00 , 0x00 
                        # , array.array('B', [0x00, 0xC2, 0x01, 0x00, 0x00, 0x00, 0x08]) )
                        , array.array('B', [0x00, 0xCA, 0x08, 0x00, 0x00, 0x00, 0x08]) )
if ret<0: raise Exception("set line state failed")

rint = lambda : random.randint(0,255)
j = {"r":rint(),"g":rint(),"b":rint(),"p":rint()}
j = {"r":255,"g":0,"b":0,"p":100}
s = json.dumps( j ) + "\r\n"
emptyarray = array.array('B', [0x00] * 64 ) 
ret = dev.write( ep_out_addr , bytes(s,"ascii") + b"\r\n"  , 100 )
print( ret )
if True:
    rets = b""
    for i in range(50):
        try:
            ret = dev.read( ep_in_addr , 64 , 100 )
        except usb.core.USBTimeoutError as e: 
            continue
        except usb.core.USBError as e:
            print( "USBError" , e )
            ret = emptyarray
            break
    
        if ret != emptyarray:
            # print( "data" , ret )
            rets += ret.tobytes()
            if rets.endswith(b"\r\n"): break

    print( rets )
# set line encoding 0x2021 (9600, 8N1)
"""
     - set line encoding: here 9600 8N1
     9600 = 0x2580 ~> 0x80, 0x25 in little endian
    115200 0x1C200 => 00 01 C2 00 => 00 C2 01 00
    
    unsigned char encoding[] = { 0x80, 0x25, 0x00, 0x00, 0x00, 0x00, 0x08 };
    rc = libusb_control_transfer(devh, 0x21, 0x20, 0, 0, encoding,
                                sizeof(encoding), 0);
"""
if False:
    dev.ctrl_transfer(0x21, 0x20, 0, 0,
                    #   array.array('B', [0x80, 0x25, 0x00, 0x00, 0x00, 0x00, 0x08]))
                    array.array('B', [0x00, 0xC2, 0x01, 0x00, 0x00, 0x00, 0x08]))

    while(True):
        dev.write(0x02, 't', interface = 1)
        try:
            print( 'Received: "%s"' % dev.read(0x83, 64, interface = 1).tostring() )
        except:
            print ('read failed')
            pass