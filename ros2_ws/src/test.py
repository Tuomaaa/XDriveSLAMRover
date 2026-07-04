import mmap, struct, time

GPIO_BASE = 0xFE200000
f = open('/dev/mem', 'r+b', buffering=0)
mem = mmap.mmap(f.fileno(), 4096, offset=GPIO_BASE)
f.close()

def rd(off):
    return struct.unpack_from('<I', mem, off)[0]
def wr(off, v):
    struct.pack_into('<I', mem, off, v)

CLK, CMD, DAT, CS = 17, 27, 22, 23

for pin in [CLK, CMD, CS]:
    reg = 0x00 + (pin//10)*4
    v = rd(reg); s = (pin%10)*3
    wr(reg, (v & ~(7<<s)) | (1<<s))

reg = 0x00 + (DAT//10)*4
v = rd(reg); s = (DAT%10)*3
wr(reg, v & ~(7<<s))
preg = 0xE4 + (DAT//16)*4
v = rd(preg); s2 = (DAT%16)*2
wr(preg, (v & ~(3<<s2)) | (1<<s2))

def high(p): wr(0x1C, 1<<p)
def low(p):  wr(0x28, 1<<p)
def readp(p): return (rd(0x34)>>p)&1

high(CLK); high(CMD); high(CS)
time.sleep(0.01)

def xfer(tx):
    rx = 0
    for i in range(8):
        if (tx>>i)&1: high(CMD)
        else: low(CMD)
        low(CLK)
        time.sleep(0.00005)
        if readp(DAT): rx |= (1<<i)   # 改：CLK LOW 时读
        high(CLK)
        time.sleep(0.00005)
    return rx

low(CS); time.sleep(0.0001)
b1 = xfer(0x01)
b2 = xfer(0x42)
b3 = xfer(0x00)
b4 = xfer(0x00)
b5 = xfer(0x00)
high(CS)

print(f'Byte1: 0x{b1:02X}  (expect 0xFF)')
print(f'Byte2: 0x{b2:02X}  (expect 0x41 or 0x73)')
print(f'Byte3: 0x{b3:02X}  (expect 0x5A)')
print(f'Byte4: 0x{b4:02X}  (btn1)')
print(f'Byte5: 0x{b5:02X}  (btn2)')
mem.close()
