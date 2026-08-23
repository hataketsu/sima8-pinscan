import struct, sys
d = open(sys.argv[1], 'rb').read()
PORTCH = 'ABF'
def pn(c):
    return f"P{PORTCH[c >> 4]}{c & 15}"

magic, phase, npins = struct.unpack_from('<III', d, 0)
pins = list(d[12:12+32])[:npins]
pu = struct.unpack_from('<III', d, 44)
pd = struct.unpack_from('<III', d, 56)
i2c_n, = struct.unpack_from('<I', d, 68)
spi_n, = struct.unpack_from('<I', d, 104)

print(f"magic=0x{magic:08X} ({'DONE' if magic==0x50534331 else 'chua xong'})  phase={phase}  npins={npins}")
print("\nCENSUS (HI=bi keo cao ngoai, LO=keo thap, --=tha noi):")
row = []
for c in pins:
    p, b = c >> 4, c & 15
    a = (pu[p] >> b) & 1
    z = (pd[p] >> b) & 1
    row.append(f"{pn(c)}={'HI' if a and z else 'LO' if not a and not z else '--'}")
for i in range(0, len(row), 8):
    print("  " + "  ".join(row[i:i+8]))

print(f"\nI2C hits: {i2c_n}")
for i in range(min(i2c_n, 8)):
    scl, sda, addr, who = d[72+i*4 : 76+i*4]
    print(f"  SCL={pn(scl)}  SDA={pn(sda)}  addr=0x{addr:02X}  WHO_AM_I=0x{who:02X}"
          + ("   <- MPU6050" if who == 0x68 else ""))

print(f"\nSPI hits: {spi_n}")
for i in range(min(spi_n, 8)):
    csn, sck, mosi, miso, val = d[108+i*8 : 113+i*8]
    print(f"  CSN={pn(csn)}  SCK={pn(sck)}  MOSI={pn(mosi)}  MISO={pn(miso)}  val=0x{val:02X}")
