from typing import List
from memory import Memory
from utils import u16, i8

class CPU:
    M_CYCLES_PER_FRAME = 17556

    def __init__(self, rom: str):
        # 16-bit registers that are made up of two smaller 8-bit registers
        # ex. HL = H(upper 8 bit register) + L(lower 8 bit register) = 16-bit register
        self.A = 0x01
        self.BC = 0x0013
        self.DE = 0x00D8
        self.HL = 0x014D

        # strictly 16-bit only
        self.SP = 0xFFFE # stack pointer
        self.PC = 0x0100 # program counter

        # CPU Flags (used to perform conditional branching AKA if statements)
        # NOTE: each represent 1 bit and together are the upper 4 bits of the F register
        self.Z = 1 # zero flag
        self.N = 0 # negative (AKA substract) flag
        self.H = 1 # half-carry flag
        self.C = 1 # carry flag

        self.mem = Memory(rom)

        # interrupt master enable: if unset, interrupts absolutely cannot happen
        self.ime = 0

    def read_F_register(self) -> int:
        return (self.Z << 3) | (self.N << 2) | (self.H << 1) | self.C

    def read_u8_imm(self) -> int:
        '''
        reads an unsigned 8-bit immediate value from the location of PC.
        This function is called by instructions that use this immediate value.

        :return: unsigned 8-bit value
        '''
        byte = self.mem.read(self.PC)
        self.PC = u16(self.PC + 1)
        return byte

    def read_u16_imm(self) -> int:
        '''
        reads an unsigned 16-bit immediate value from the location of PC.
        This function is called by instructions that use this immediate value.

        :return: unsigned 16-bit value
        '''
        lsb = self.mem.read(self.PC)
        self.PC = u16(self.PC + 1)
        msb = self.mem.read(self.PC)
        self.PC = u16(self.PC + 1)
        return (msb << 8) | lsb

    def execute_prefixed(self) -> int:
        opcode = self.mem.read(self.PC)
        self.PC = u16(self.PC + 1)
        # TODO
        return 1

    # https://gekkio.fi/files/gb-docs/gbctr.pdf
    def execute(self) -> int:
        # each instruction starts by reading the byte at PC which represents the opcode
        # reference opcode table here: https://gbdev.io/gb-opcodes/optables/
        opcode = self.mem.read(self.PC)
        self.PC = u16(self.PC + 1)

        match opcode:
            case 0x00: # NOP (pg. 126)
                print("NOP")
                return 1

            case 0xC3: # JP nn (pg. 110)
                nn = self.read_u16_imm()
                self.PC = nn
                print("JP", hex(nn))
                return 4

            case 0xF3: # DI (pg. 124)
                self.ime = 0
                print("DI")
                return 1

            case 0x31: # LD SP, nn (pg. 40)
                nn = self.read_u16_imm()
                self.SP = nn
                print("LD SP,", hex(nn))
                return 3

            case 0xEA: # LD (nn), A (pg. 31)
                nn = self.read_u16_imm()
                self.mem.write(nn, self.A)
                print(f"LD [{hex(nn)}], A")
                return 4

            case 0x3E: # LD A, n (pg. 22)
                n = self.read_u8_imm()
                self.A = n
                print("LD A,", hex(n))
                return 2

            case 0xE0: # LDH (n), A (pg. 35)
                n = self.read_u8_imm()
                self.mem.write(0xFF00 | n, self.A)
                print(f"LDH [{hex(0xFF00 | n)}], A")
                return 3

            case 0x21: # LD HL, nn (pg. 40)
                nn = self.read_u16_imm()
                self.HL = nn
                print("LD HL,", hex(nn))
                return 3

            case 0xCD: # CALL nn (pg. 117)
                nn = self.read_u16_imm()
                self.SP = u16(self.SP - 1)
                self.mem.write(self.SP, self.PC >> 8)
                self.SP = u16(self.SP - 1)
                self.mem.write(self.SP, self.PC & 0xFF)
                self.PC = nn
                print("CALL", hex(nn))
                return 6

            case 0x7D: # LD A, L (pg. 21)
                self.A = self.HL & 0xFF
                print("LD A, L")
                return 1

            case 0x7C: # LD A, H (pg. 21)
                self.A = self.HL >> 8
                print("LD A, H")
                return 1

            case 0x18: # JR e (pg. 114)
                e = i8(self.read_u8_imm())
                self.PC = u16(self.PC + e)
                print("JR", hex(e))
                return 3

            case 0xC9: # RET (pg. 120)
                lsb = self.mem.read(self.SP)
                self.SP = u16(self.SP + 1)
                msb = self.mem.read(self.SP)
                self.SP = u16(self.SP + 1)
                self.PC = (msb << 8) | lsb
                print("RET")
                return 4

            case 0xE5: # PUSH HL (pg. 43)
                self.SP = u16(self.SP - 1)
                self.mem.write(self.SP, self.HL >> 8)
                self.SP = u16(self.SP - 1)
                self.mem.write(self.SP, self.HL & 0xFF)
                print("PUSH HL")
                return 4

            case 0xE1: # POP HL (pg. 44)
                lsb = self.mem.read(self.SP)
                self.SP = u16(self.SP + 1)
                msb = self.mem.read(self.SP)
                self.SP = u16(self.SP + 1)
                self.HL = (msb << 8) | lsb
                print("POP HL")
                return 3

            case 0xF5: # PUSH AF (pg. 43)
                self.SP = u16(self.SP - 1)
                self.mem.write(self.SP, self.A)
                self.SP = u16(self.SP - 1)
                self.mem.write(self.SP, self.read_F_register())
                print("PUSH AF")
                return 4

            case 0x23: # INC HL (pg. 78)
                self.HL = u16(self.HL + 1)
                print("INC HL")
                return 2

            case 0x2A: # LD A, (HL+) (pg. 38)
                self.A = self.mem.read(self.HL)
                self.HL = u16(self.HL + 1)
                print("LD A, (HL+)")
                return 2

            case 0xF1: # POP AF (pg. 44)
                lsb = self.mem.read(self.SP)
                self.SP = u16(self.SP + 1)
                msb = self.mem.read(self.SP)
                self.SP = u16(self.SP + 1)
                self.A = msb
                self.Z = (lsb >> 3) & 1
                self.N = (lsb >> 2) & 1
                self.H = (lsb >> 1) & 1
                self.C = lsb & 1
                print("POP AF")
                return 3

            case 0xC5: # PUSH BC (pg. 43)
                self.SP = u16(self.SP - 1)
                self.mem.write(self.SP, self.BC >> 8)
                self.SP = u16(self.SP - 1)
                self.mem.write(self.SP, self.BC & 0xFF)
                print("PUSH BC")
                return 4

            case 0x01: # LD BC, nn (pg. 40)
                nn = self.read_u16_imm()
                self.BC = nn
                print("LD BC,", hex(nn))
                return 3

            case 0x03: # INC BC (pg. 78)
                self.BC = u16(self.BC + 1)
                print("INC BC")
                return 2

            case 0x78: # LD A, B (pg. 21)
                self.A = self.BC >> 8
                print("LD A, B")
                return 1

            case 0xB1: # OR A, C (pg. 68)
                self.A |= (self.BC & 0xFF)
                self.Z = self.A == 0
                self.N = 0
                self.H = 0
                self.C = 0
                print("OR A, C")
                return 1

            case _:
                print("undefined opcode:", hex(opcode))
                exit(1)

    def render_frame(self) -> List[List[int]]:
        # each frame takes a fixed length of "time" to render and the way
        # we represent this "time" is through CPU M-cycles. An instruction
        # that is executed can vary between 1-N M-cycles (machine cycles) and
        # we sum the total amount of M-cycles processed per frame here
        m_cycles_ran = 0

        # After 17556 M-cycles, a frame is rendered!
        while m_cycles_ran < self.M_CYCLES_PER_FRAME:
            m_cycles_ran += self.execute()

        return []
