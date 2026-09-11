# ============================================================
# CAN + Timer LED demo, with load-delay slots.
#
# WORKAROUND FOR A CPU FORWARDING BUG
#   top_level_riscv.v forwards `execute_result_delay` for a
#   register produced two instructions earlier. For a LOAD that
#   register holds the memory ADDRESS, not the loaded data --
#   `mem_to_reg_delay` exists but is never consulted. hazard_unit
#   only stalls the distance-1 case, so distance-2 silently
#   returns the wrong value.
#
#   Every load here is therefore followed by three NOPs, so the
#   value is consumed from the write-back stage where the
#   forwarding is correct. Cheap, and it keeps the verified CPU
#   RTL untouched.
# ============================================================
def I(imm,rs1,f3,rd,op):    return ((imm&0xFFF)<<20)|(rs1<<15)|(f3<<12)|(rd<<7)|op
def R(f7,rs2,rs1,f3,rd,op): return (f7<<25)|(rs2<<20)|(rs1<<15)|(f3<<12)|(rd<<7)|op
def S(imm,rs2,rs1,f3,op):
    i=imm&0xFFF
    return ((i>>5)<<25)|(rs2<<20)|(rs1<<15)|(f3<<12)|((i&0x1F)<<7)|op
def U(imm,rd,op): return ((imm&0xFFFFF)<<12)|(rd<<7)|op
def B(imm,rs2,rs1,f3,op):
    i=imm&0x1FFF
    return (((i>>12)&1)<<31)|(((i>>5)&0x3F)<<25)|(rs2<<20)|(rs1<<15)|(f3<<12)|\
           (((i>>1)&0xF)<<8)|(((i>>11)&1)<<7)|op
def J(imm,rd,op):
    i=imm&0x1FFFFF
    return (((i>>20)&1)<<31)|(((i>>1)&0x3FF)<<21)|(((i>>11)&1)<<20)|\
           (((i>>12)&0xFF)<<12)|(rd<<7)|op

lui=lambda rd,i:U(i,rd,0x37); addi=lambda rd,a,i:I(i,a,0,rd,0x13)
andi=lambda rd,a,i:I(i,a,7,rd,0x13); slli=lambda rd,a,s:I(s,a,1,rd,0x13)
xor=lambda rd,a,b:R(0,b,a,4,rd,0x33)
sw=lambda r2,i,r1:S(i,r2,r1,2,0x23)
beq=lambda a,b,o:B(o,b,a,0,0x63); jal=lambda rd,o:J(o,rd,0x6F)
csrrw=lambda rd,c,a:I(c,a,1,rd,0x73); csrrs=lambda rd,c,a:I(c,a,2,rd,0x73)
MRET=0x30200073; NOP=addi(0,0,0)

def lwd(rd,i,r1):
    """Load followed by three delay slots. See header."""
    return [I(i,r1,2,rd,0x03), NOP, NOP, NOP]

def split(v):
    lo=v&0xFFF
    return (((v>>12)+1)&0xFFFFF, lo-0x1000) if lo>=0x800 else ((v>>12)&0xFFFFF, lo)

GPIO,CAN,TMR,INTC=0x40000000,0x40070000,0x40010000,0x40080000
G_DIR,G_OUT,G_IN=0x00,0x04,0x08
C_CTRL,C_BTIME,C_TXID,C_TXD0,C_TXD1,C_TXCTRL=0x00,0x04,0x0C,0x10,0x14,0x18
C_RXID,C_RXCTRL,C_IE,C_IP=0x1C,0x28,0x2C,0x30
T_CTRL,T_PRE,T_CMP,T_STAT=0x00,0x04,0x08,0x10
I_MASK,I_PEND,I_CAUSE=0x00,0x04,0x08

# x1 GPIO   x5 CAN   x7 TIMER   x8 INTC   x26 tick counter
m=[]
m+=[lui(1,GPIO>>12), addi(2,0,-1), sw(2,G_DIR,1), sw(0,G_OUT,1)]
m+=[lui(5,CAN>>12), addi(6,0,3), sw(6,C_CTRL,5)]
hi,lo=split(0x00122200); m+=[lui(6,hi), addi(6,6,lo), sw(6,C_BTIME,5)]
m+=[addi(6,0,7), sw(6,C_IE,5)]
m+=[lui(7,TMR>>12), addi(9,0,799), sw(9,T_PRE,7)]
hi,lo=split(49999);  m+=[lui(10,hi), addi(10,10,lo), sw(10,T_CMP,7)]
m+=[addi(11,0,1), sw(11,T_STAT,7)]
m+=[lui(8,INTC>>12), addi(12,0,0x1FF), sw(12,I_PEND,8)]
m+=[addi(13,0,0x41), sw(13,I_MASK,8)]
m+=[addi(14,0,0x100), csrrw(0,0x305,14)]
m+=[addi(15,0,1), slli(15,15,11), csrrs(0,0x304,15)]
m+=[addi(16,0,8), csrrs(0,0x300,16)]
m+=[addi(26,0,0)]
m+=[addi(17,0,7), sw(17,T_CTRL,7)]
m+=[addi(27,27,1), jal(0,-4)]

H=64
assert len(m)<=H, f"main {len(m)}w overruns handler at {H}"

h=[]
h+=[csrrs(20,0x342,0)]
h+=lwd(21,I_CAUSE,8)          # x21 = INTC.CAUSE   (+3 delay slots)
h+=[addi(9,0,6)]
BEQ=len(h); h+=[0]            # beq x21,x9 -> TIMER_ISR
# ---------- CAN RX : IRQ 0 ----------
h+=lwd(22,C_RXID,5)           # received ID        (+3 delay slots)
h+=[sw(22,G_OUT,1)]           # -> LEDs
h+=[I(C_RXCTRL,5,2,23,0x03)]  # pops the RX FIFO; result unused, no slots needed
h+=[addi(28,0,7), sw(28,C_IP,5)]
h+=[addi(29,0,1), sw(29,I_PEND,8)]
h+=[MRET]
TI=len(h)
# ---------- TIMER : IRQ 6 ----------
h+=[addi(26,26,1)]
h+=lwd(30,G_IN,1)             # slide switches     (+3 delay slots)
h+=[xor(31,26,30), andi(31,31,0x7FF)]
h+=[sw(31,C_TXID,5), sw(26,C_TXD0,5), sw(30,C_TXD1,5)]
h+=[addi(17,0,8), sw(17,C_TXCTRL,5)]
h+=[addi(24,0,1), sw(24,T_STAT,7)]
h+=[addi(29,0,0x40), sw(29,I_PEND,8)]
h+=[MRET]
h[BEQ]=beq(21,9,(TI-BEQ)*4)

w=m+[NOP]*(H-len(m))+h+[NOP]*4
open("program.mem","w").write("\n".join(f"{x:08X}" for x in w)+"\n")
print(f"main={len(m)}w handler={len(h)}w total={len(w)}w")
print(f"TIMER_ISR at 0x{(H+TI)*4:03X}, beq offset +{(TI-BEQ)*4}")
