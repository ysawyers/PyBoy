from typing import List
from memory import Memory
from utils import u16, i8, u8

class CPU:
    M_CYCLES_PER_FRAME = 17556

    def __init__(self, rom: str | None):
        # 16-bit registers that are made up of two smaller 8-bit registers
        # ex. HL = H(upper 8 bit register) + L(lower 8 bit register) = 16-bit register
        self.AF = 0x0100
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

        # if set, delays setting ime by 1 M-Cycle
        self.delayed_ime_enable = False

    def flag_bits(self) -> int:
        return (self.Z << 7) | (self.N << 6) | (self.H << 5) | (self.C << 4)

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

    def execute_prefixed(self):
        opcode = self.mem.read(self.PC)
        self.PC = u16(self.PC + 1)
        # TODO

    # https://gekkio.fi/files/gb-docs/gbctr.pdf
    # https://meganesu.github.io/generate-gb-opcodes/
    def execute(self):
        # each instruction starts by reading the byte at PC which represents the opcode
        # reference opcode table here: https://gbdev.io/gb-opcodes/optables/
        opcode = self.mem.read(self.PC)
        self.PC = u16(self.PC + 1)

        match opcode:
            case 0x00: # NOP (pg. 126)
                pass

            case 0x02: # LD (BC), A (pg. 28)
                self.mem.write(self.BC, self.AF >> 8)

            case 0xC3: # JP nn (pg. 110)
                nn = self.read_u16_imm()
                self.PC = nn
                self.mem.tick() # internal cycle

            case 0xF3: # DI (pg. 124)
                self.ime = 0

            case 0x31: # LD SP, nn (pg. 40)
                nn = self.read_u16_imm()
                self.SP = nn

            case 0xEA: # LD (nn), A (pg. 31)
                nn = self.read_u16_imm()
                self.mem.write(nn, self.AF >> 8)

            case 0x3E: # LD A, n (pg. 22)
                n = self.read_u8_imm()
                self.AF = self.AF & 0x00FF | (n << 8)

            case 0xE0: # LDH (n), A (pg. 35)
                n = self.read_u8_imm()
                self.mem.write(0xFF00 | n, (self.AF >> 8))

            case 0x21: # LD HL, nn (pg. 40)
                nn = self.read_u16_imm()
                self.HL = nn

            case 0xCD: # CALL nn (pg. 117)
                nn = self.read_u16_imm()
                self.mem.tick() # internal cycle
                self.SP = u16(self.SP - 1)
                self.mem.write(self.SP, self.PC >> 8)
                self.SP = u16(self.SP - 1)
                self.mem.write(self.SP, self.PC & 0xFF)
                self.PC = nn

            case 0x7D: # LD A, L (pg. 21)
                self.AF = (self.AF & 0x00FF) | ((self.HL & 0xFF) << 8)

            case 0x7C: # LD A, H (pg. 21)
                self.AF = (self.AF & 0x00FF) | ((self.HL >> 8) << 8)

            case 0x18: # JR e (pg. 114)
                e = i8(self.read_u8_imm())
                self.PC = u16(self.PC + e)
                self.mem.tick() # internal cycle

            case 0xC9: # RET (pg. 120)
                lsb = self.mem.read(self.SP)
                self.SP = u16(self.SP + 1)
                msb = self.mem.read(self.SP)
                self.SP = u16(self.SP + 1)
                self.PC = (msb << 8) | lsb
                self.mem.tick() # internal cycle

            case 0xE5: # PUSH HL (pg. 43)
                self.mem.tick() # internal cycle
                self.SP = u16(self.SP - 1)
                self.mem.write(self.SP, self.HL >> 8)
                self.SP = u16(self.SP - 1)
                self.mem.write(self.SP, self.HL & 0xFF)

            case 0xE1: # POP HL (pg. 44)
                lsb = self.mem.read(self.SP)
                self.SP = u16(self.SP + 1)
                msb = self.mem.read(self.SP)
                self.SP = u16(self.SP + 1)
                self.HL = (msb << 8) | lsb

            case 0xF5: # PUSH AF (pg. 43)
                self.mem.tick() # internal cycle
                self.SP = u16(self.SP - 1)
                self.mem.write(self.SP, self.AF >> 8)
                self.SP = u16(self.SP - 1)
                self.mem.write(self.SP, self.flag_bits() | (self.AF & 0x0F))

            case 0x23: # INC HL (pg. 78)
                self.HL = u16(self.HL + 1)
                self.mem.tick() # internal cycle

            case 0x2A: # LD A, (HL+) (pg. 38)
                self.AF = (self.AF & 0x00FF) | (self.mem.read(self.HL) << 8)
                self.HL = u16(self.HL + 1)

            case 0xF1: # POP AF (pg. 44)
                lsb = self.mem.read(self.SP)
                self.SP = u16(self.SP + 1)
                msb = self.mem.read(self.SP)
                self.SP = u16(self.SP + 1)
                self.Z = (lsb >> 7) & 1
                self.N = (lsb >> 6) & 1
                self.H = (lsb >> 5) & 1
                self.C = (lsb >> 4) & 1
                self.AF = (msb << 8) | self.flag_bits()

            case 0xC5: # PUSH BC (pg. 43)
                self.mem.tick() # internal cycle
                self.SP = u16(self.SP - 1)
                self.mem.write(self.SP, self.BC >> 8)
                self.SP = u16(self.SP - 1)
                self.mem.write(self.SP, self.BC & 0xFF)

            case 0x01: # LD BC, nn (pg. 40)
                nn = self.read_u16_imm()
                self.BC = nn

            case 0x03: # INC BC (pg. 78)
                self.BC = u16(self.BC + 1)
                self.mem.tick() # internal cycle

            case 0x78: # LD A, B (pg. 21)
                self.AF = (self.AF & 0x00FF) | ((self.BC >> 8) << 8)

            case 0xB1: # OR A, C (pg. 68)
                self.AF |= ((self.BC & 0xFF) << 8)
                self.Z = (self.AF >> 8) == 0
                self.N = 0
                self.H = 0
                self.C = 0

            case 0x28: # JR Z, e (pg. 115)
                e = i8(self.mem.read(self.PC))
                self.PC = u16(self.PC + 1)
                if (self.Z):
                    self.PC = u16(self.PC + e)
                    self.mem.tick() # internal cycle

            case 0xF0: # LDH A, (n) (pg. 34)
                n = self.read_u8_imm()
                self.AF = (self.AF & 0x00FF) | (self.mem.read(0xFF00 | n) << 8)

            case 0xFE: # CP n (pg. 60)
                n = self.read_u8_imm()
                v = u8((self.AF >> 8) - n)
                self.Z = v == 0
                self.N = 1
                self.H = ((((self.AF >> 8) & 0xF) - (n & 0xF)) & 0x10) == 0x10
                self.C = (((self.AF >> 8) - n) & 0x100) == 0x100

            case 0x20: # JR NZ, e (pg. 115)
                e = i8(self.mem.read(self.PC))
                self.PC = u16(self.PC + 1)
                if (not self.Z):
                    self.PC = u16(self.PC + e)
                    self.mem.tick() # internal cycle

            case 0xC1: # POP BC (pg. 44)
                lsb = self.mem.read(self.SP)
                self.SP = u16(self.SP + 1)
                msb = self.mem.read(self.SP)
                self.SP = u16(self.SP + 1)
                self.BC = (msb << 8) | lsb

            case 0xFA: # LD A, (nn) (pg. 30)
                nn = self.read_u16_imm()
                self.AF = (self.AF & 0x00FF) | (self.mem.read(nn) << 8)

            case 0xE6: # AND n (pg. 67)
                n = self.read_u8_imm()
                self.AF &= (n << 8) | 0xFF
                self.Z = (self.AF >> 8) == 0
                self.N = 0
                self.H = 1
                self.C = 0

            case 0xC4: # CALL NZ, nn (pg. 118)
                nn = self.read_u16_imm()
                if (not self.Z):
                    self.mem.tick() # internal cycle
                    self.SP = u16(self.SP - 1)
                    self.mem.write(self.SP, self.PC >> 8)
                    self.SP = u16(self.SP - 1)
                    self.mem.write(self.SP, self.PC & 0xFF)
                    self.PC = nn

            case 0x06: # LD B, n (pg. 22)
                n = self.read_u8_imm()
                self.BC = (self.BC & 0x00FF) | (n << 8)

            case 0x77: # LD (HL), A (pg. 24)
                self.mem.write(self.HL, self.AF >> 8)

            case 0x2C: # INC L (pg. 61)
                v = u8((self.HL & 0xFF) + 1)
                self.Z = v == 0
                self.N = 0
                self.H = (((self.HL & 0xF) + 1) & 0x10) == 0x10
                self.HL = (self.HL & 0xFF00) | v

            case 0x24: # INC H (pg. 61)
                v = u8((self.HL >> 8) + 1)
                self.Z = v == 0
                self.N = 0
                self.H = ((((self.HL >> 8) & 0xF) + 1) & 0x10) == 0x10
                self.HL = (v << 8) | (self.HL & 0x00FF)

            case 0x05: # DEC B (pg. 63)
                v = u8((self.BC >> 8) - 1)
                self.Z = v == 0
                self.N = 1
                self.H = ((((self.BC >> 8) & 0xF) - (1 & 0xF)) & 0x10) == 0x10
                self.BC = (v << 8) | (self.BC & 0x00FF)

            case 0x0E: # LD C, n (pg. 22)
                n = self.read_u8_imm()
                self.BC = (self.BC & 0xFF00) | n

            case 0x11: # LD DE, nn (pg. 40)
                nn = self.read_u16_imm()
                self.DE = nn

            case 0x1A: # LD A, (DE) (pg. 27)
                v = self.mem.read(self.DE)
                self.AF = (v << 8) | (self.AF & 0x00FF)

            case 0x13: # INC DE (pg. 78)
                self.DE = u16(self.DE + 1)
                self.mem.tick() # internal cycle

            case 0xA9: # XOR A, C (pg. 71)
                self.AF = (((self.AF >> 8) ^ (self.BC & 0xFF)) << 8) | (self.AF & 0x00FF)
                self.Z = (self.AF >> 8) == 0
                self.N = 0
                self.H = 0
                self.C = 0

            case 0x22: # LD (HL+), A (pg. 39)
                self.mem.write(self.HL, self.AF >> 8)
                self.HL = u16(self.HL + 1)

            case 0xC6: # ADD A, n (pg. 48)
                n = self.read_u8_imm()
                v = u8((self.AF >> 8) + n)
                self.Z = v == 0
                self.N = 0
                self.H = ((((self.AF >> 8) & 0xF) + (n & 0xF)) & 0x10) == 0x10
                self.C = (((self.AF >> 8) + n) & 0x100) == 0x100
                self.AF = (v << 8) | (self.AF & 0x00FF)

            case 0x32: # LD (HL-), A
                self.mem.write(self.HL, self.AF >> 8)
                self.HL = u16(self.HL - 1)

            case 0xD6: # SUB A, n (pg. 54)
                n = self.read_u8_imm()
                v = u8((self.AF >> 8) - n)
                self.Z = v == 0
                self.N = 1
                self.H = ((((self.AF >> 8) & 0xF) - (n & 0xF)) & 0x10) == 0x10
                self.C = (((self.AF >> 8) - n) & 0x100) == 0x100
                self.AF = (v << 8) | (self.AF & 0x00FF)

            case 0xB7: # OR A, A (pg. 68)
                self.Z = (self.AF >> 8) == 0
                self.N = 0
                self.H = 0
                self.C = 0

            case 0xD5: # PUSH DE (pg. 43)
                self.mem.tick() # internal cycle
                self.SP = u16(self.SP - 1)
                self.mem.write(self.SP, self.DE >> 8)
                self.SP = u16(self.SP - 1)
                self.mem.write(self.SP, self.DE & 0xFF)

            case 0x46: # LD B, (HL) (pg. 23)
                n = self.mem.read(self.HL)
                self.BC = (n << 8) | (self.BC & 0x00FF)

            case 0x2D: # DEC L (pg. 63)
                v = u8((self.HL & 0xFF) - 1)
                self.Z = v == 0
                self.N = 1
                self.H = (((self.HL & 0xF) - 1) & 0x10) == 0x10
                self.HL = (self.HL & 0xFF00) | v

            case 0x4E: # LD C, (HL) (pg. 23)
                n = self.mem.read(self.HL)
                self.BC = (self.BC & 0xFF00) | n

            case 0x56: # LD D, (HL) (pg. 23)
                n = self.mem.read(self.HL)
                self.DE = (n << 8) | (self.DE & 0x00FF)

            case 0xAE: # XOR A, (HL)
                n = self.mem.read(self.HL)
                v = (self.AF >> 8) ^ n
                self.AF = (v << 8) | (self.AF & 0x00FF)
                self.Z = v == 0
                self.N = 0
                self.H = 0
                self.C = 0

            case 0x26: # LD H, n
                n = self.read_u8_imm()
                self.HL = (n << 8) | (self.HL & 0x00FF)

            case 0x4F: # LD C, A
                self.BC = (self.BC & 0xFF00) | (self.AF >> 8)

            case 0xD4: # CALL NC, nn
                nn = self.read_u16_imm()
                if (not self.C):
                    self.mem.tick() # internal cycle
                    self.SP = u16(self.SP - 1)
                    self.mem.write(self.SP, self.PC >> 8)
                    self.SP = u16(self.SP - 1)
                    self.mem.write(self.SP, self.PC & 0xFF)
                    self.PC = nn

            case 0xA3: # AND A, E
                v = ((self.AF >> 8) & (self.DE & 0xFF))
                self.AF = (v << 8) | (self.AF & 0x00FF)
                self.Z = v == 0
                self.N = 0
                self.H = 1
                self.C = 0

            case 0xE2: # LD (FF00+C), A
                self.mem.write(0xFF00 | (self.BC & 0xFF), self.AF >> 8)

            case 0xBB: # CP A, E
                v = u8((self.AF >> 8) - (self.DE & 0xFF))
                self.Z = v == 0
                self.N = 1
                self.H = ((((self.AF >> 8) & 0xF) - ((self.DE & 0xFF) & 0xF)) & 0x10) == 0x10
                self.C = (((self.AF >> 8) - (self.DE & 0xFF)) & 0x100) == 0x100

            case 0xB9: # CP A, C
                v = u8((self.AF >> 8) - (self.BC & 0xFF))
                self.Z = v == 0
                self.N = 1
                self.H = ((((self.AF >> 8) & 0xF) - ((self.BC & 0xFF) & 0xF)) & 0x10) == 0x10
                self.C = (((self.AF >> 8) - (self.BC & 0xFF)) & 0x100) == 0x100

            case 0x9C: # SBC A, H
                v = u8((self.AF >> 8) - (self.HL >> 8) - self.C)
                self.Z = v == 0
                self.N = 1
                self.H = ((((self.AF >> 8) & 0xF) - ((self.HL >> 8) & 0xF) - self.C) & 0x10) == 0x10
                self.C = (((self.AF >> 8) - (self.HL >> 8) - self.C) & 0x100) == 0x100
                self.AF = (v << 8) | (self.AF & 0x00FF)

            case 0x98: # SBC A, B
                v = u8((self.AF >> 8) - (self.BC >> 8) - self.C)
                self.Z = v == 0
                self.N = 1
                self.H = ((((self.AF >> 8) & 0xF) - ((self.BC >> 8) & 0xF) - self.C) & 0x10) == 0x10
                self.C = (((self.AF >> 8) - (self.BC >> 8) - self.C) & 0x100) == 0x100
                self.AF = (v << 8) | (self.AF & 0x00FF)

            case 0x8E: # ADC A, (HL)
                n = self.mem.read(self.HL)
                v = u8((self.AF >> 8) + n + self.C)
                self.Z = v == 0
                self.N = 0
                self.H = ((((self.AF >> 8) & 0xF) + (n & 0xF) + self.C) & 0x10) == 0x10
                self.C = (((self.AF >> 8) + n + self.C) & 0x100) == 0x100
                self.AF = (v << 8) | (self.AF & 0x00FF)

            case 0xF8: # LD HL, SP+i8
                e = i8(self.read_u8_imm())
                self.HL = u16(self.SP + e)
                self.Z = 0
                self.N = 0
                self.H = (((self.SP & 0xF) + (e & 0xF)) & 0x10) == 0x10
                self.C = (((self.SP & 0xFF) + (e & 0xFF)) & 0x100) == 0x100
                self.mem.tick() # internal cycle

            case 0x61: # LD H, C
                self.HL = ((self.BC & 0xFF) << 8) | (self.HL & 0x00FF)

            case 0x36: # LD (HL), u8
                n = self.read_u8_imm()
                self.mem.write(self.HL, n)

            case 0x41: # LD B, C
                self.BC = ((self.BC & 0xFF) << 8) | (self.BC & 0x00FF)

            case 0xD8: # RET C
                self.mem.tick() # internal cycle
                if self.C:
                    lsb = self.mem.read(self.SP)
                    self.SP = u16(self.SP + 1)
                    msb = self.mem.read(self.SP)
                    self.SP = u16(self.SP + 1)
                    self.PC = (msb << 8) | lsb
                    self.mem.tick() # internal cycle

            case 0xDC: # CALL C, nn
                nn = self.read_u16_imm()
                if (self.C):
                    self.mem.tick() # internal cycle
                    self.SP = u16(self.SP - 1)
                    self.mem.write(self.SP, self.PC >> 8)
                    self.SP = u16(self.SP - 1)
                    self.mem.write(self.SP, self.PC & 0xFF)
                    self.PC = nn

            case 0x16: # LD D, n
                n = self.read_u8_imm()
                self.DE = (n << 8) | (self.DE & 0x00FF)

            case 0xAD: # XOR A, L
                self.AF = (((self.AF >> 8) ^ (self.HL & 0xFF)) << 8) | (self.AF & 0x00FF)
                self.Z = (self.AF >> 8) == 0
                self.N = 0
                self.H = 0
                self.C = 0

            case 0x57: # LD D, A
                self.DE = ((self.AF >> 8) << 8) | (self.DE & 0x00FF)

            case 0xEE: # XOR A, n
                n = self.read_u8_imm()
                self.AF = (((self.AF >> 8) ^ n) << 8) | (self.AF & 0x00FF)
                self.Z = (self.AF >> 8) == 0
                self.N = 0
                self.H = 0
                self.C = 0

            case 0xB5: # OR A, L
                self.AF |= ((self.HL & 0xFF) << 8)
                self.Z = (self.AF >> 8) == 0
                self.N = 0
                self.H = 0
                self.C = 0

            case 0x94: # SUB A, H
                v = u8((self.AF >> 8) - (self.HL >> 8))
                self.Z = v == 0
                self.N = 1
                self.H = ((((self.AF >> 8) & 0xF) - ((self.HL >> 8) & 0xF)) & 0x10) == 0x10
                self.C = (((self.AF >> 8) - (self.HL >> 8)) & 0x100) == 0x100
                self.AF = (v << 8) | (self.AF & 0x00FF)

            case 0x82: # ADD A, D
                v = u8((self.AF >> 8) + (self.DE >> 8))
                self.Z = v == 0
                self.N = 0
                self.H = ((((self.AF >> 8) & 0xF) + ((self.DE >> 8) & 0xF)) & 0x10) == 0x10
                self.C = (((self.AF >> 8) + (self.DE >> 8)) & 0x100) == 0x100
                self.AF = (v << 8) | (self.AF & 0x00FF)

            case 0x6F: # LD L, A 
                self.HL = (self.HL & 0xFF00) | (self.AF >> 8)

            case 0x3A: # LD A, (HL-)
                n = self.mem.read(self.HL)
                self.AF = (n << 8) | self.AF & 0x00FF                
                self.HL = u16(self.HL - 1)

            case 0xC2: # JP NZ, nn
                nn = self.read_u16_imm()
                if (not self.Z):
                    self.PC = nn
                    self.mem.tick() # internal cycle

            case 0x83: # ADD A, E
                v = u8((self.AF >> 8) + (self.DE & 0xFF))
                self.Z = v == 0
                self.N = 0
                self.H = ((((self.AF >> 8) & 0xF) + ((self.DE & 0xFF) & 0xF)) & 0x10) == 0x10
                self.C = (((self.AF >> 8) + (self.DE & 0xFF)) & 0x100) == 0x100
                self.AF = (v << 8) | (self.AF & 0x00FF)

            case 0x7A: # LD A, D
                self.AF = ((self.DE >> 8) << 8) | (self.AF & 0x00FF)

            case 0x95: # SUB A, L
                v = u8((self.AF >> 8) - (self.HL & 0xFF))
                self.Z = v == 0
                self.N = 1
                self.H = ((((self.AF >> 8) & 0xF) - ((self.HL & 0xFF) & 0xF)) & 0x10) == 0x10
                self.C = (((self.AF >> 8) - (self.HL & 0xFF)) & 0x100) == 0x100
                self.AF = (v << 8) | (self.AF & 0x00FF)

            case 0x2F: # CPL
                self.AF = (((self.AF >> 8) ^ 0xFF) << 8) | (self.AF & 0x00FF)
                self.N = 1
                self.H = 1

            case 0xB4: # OR A, H
                self.AF |= ((self.HL >> 8) << 8)
                self.Z = (self.AF >> 8) == 0
                self.N = 0
                self.H = 0
                self.C = 0

            case 0x17: # RLA
                v = (self.AF >> 8) << 1
                self.AF = (u8(v | self.C) << 8) | (self.AF & 0x00FF)
                self.Z = 0
                self.N = 0
                self.H = 0
                self.C = (v & 0x100) == 0x100

            case 0x40: # LD B, B
                pass

            case 0xD9: # RETI
                lsb = self.mem.read(self.SP)
                self.SP = u16(self.SP + 1)
                msb = self.mem.read(self.SP)
                self.SP = u16(self.SP + 1)
                self.PC = (msb << 8) | lsb
                self.ime = 1
                self.mem.tick() # internal cycle

            case 0x37: # SCF
                self.N = 0
                self.H = 0
                self.C = 1

            case 0xCE: # ADC n
                n = self.read_u8_imm()
                v = u8((self.AF >> 8) + n + self.C)
                self.Z = v == 0
                self.N = 0
                self.H = ((((self.AF >> 8) & 0xF) + (n & 0xF) + self.C) & 0x10) == 0x10
                self.C = (((self.AF >> 8) + n + self.C) & 0x100) == 0x100
                self.AF = (v << 8) | (self.AF & 0x00FF)

            case 0xF9: # LD SP, HL
                self.SP = self.HL
                self.mem.tick() # internal cycle

            case 0x60: # LD H, B
                self.HL = ((self.BC >> 8) << 8) | (self.HL & 0x00FF)

            case 0xFB: # EI
                self.delayed_ime_enable = True

            case 0x8D: # ADC A, L
                v = u8((self.AF >> 8) + (self.HL & 0xFF) + self.C)
                self.Z = v == 0
                self.N = 0
                self.H = ((((self.AF >> 8) & 0xF) + (self.HL & 0xF) + self.C) & 0x10) == 0x10
                self.C = (((self.AF >> 8) + (self.HL & 0xFF) + self.C) & 0x100) == 0x100
                self.AF = (v << 8) | (self.AF & 0x00FF)

            case 0x99: # SBC A, C
                v = u8((self.AF >> 8) - (self.BC & 0xFF) - self.C)
                self.Z = v == 0
                self.N = 1
                self.H = ((((self.AF >> 8) & 0xF) - ((self.BC & 0xFF) & 0xF) - self.C) & 0x10) == 0x10
                self.C = (((self.AF >> 8) - (self.BC & 0xFF) - self.C) & 0x100) == 0x100
                self.AF = (v << 8) | (self.AF & 0x00FF)

            case 0x76: # HALT
                print("TODO HALT")
                exit(1)

            case 0x9B: # SBC A, E
                v = u8((self.AF >> 8) - (self.DE & 0xFF) - self.C)
                self.Z = v == 0
                self.N = 1
                self.H = ((((self.AF >> 8) & 0xF) - ((self.DE & 0xFF) & 0xF) - self.C) & 0x10) == 0x10
                self.C = (((self.AF >> 8) - (self.DE & 0xFF) - self.C) & 0x100) == 0x100
                self.AF = (v << 8) | (self.AF & 0x00FF)

            case 0xB8: # CP A, B
                v = u8((self.AF >> 8) - (self.BC >> 8))
                self.Z = v == 0
                self.N = 1
                self.H = ((((self.AF >> 8) & 0xF) - ((self.BC >> 8) & 0xF)) & 0x10) == 0x10
                self.C = (((self.AF >> 8) - (self.BC >> 8)) & 0x100) == 0x100

            case 0xBC: # CP A, H
                v = u8((self.AF >> 8) - (self.HL >> 8))
                self.Z = v == 0
                self.N = 1
                self.H = ((((self.AF >> 8) & 0xF) - ((self.HL >> 8) & 0xF)) & 0x10) == 0x10
                self.C = (((self.AF >> 8) - (self.HL >> 8)) & 0x100) == 0x100

            case 0x5A: # LD E, D
                self.DE = (self.DE & 0xFF00) | (self.DE >> 8)

            case 0x0F: # RRCA
                self.C = (self.AF >> 8) & 1
                self.AF = (u8(((self.AF >> 8) >> 1) | (self.C << 7)) << 8) | (self.AF & 0x00FF)
                self.Z = 0
                self.N = 0
                self.H = 0

            case 0xA2: # AND A, D
                v = ((self.AF >> 8) & (self.DE >> 8))
                self.AF = (v << 8) | (self.AF & 0x00FF)
                self.Z = v == 0
                self.N = 0
                self.H = 1
                self.C = 0

            case 0x47: # LD B, A
                self.BC = ((self.AF >> 8) << 8) | (self.BC & 0x00FF)

            case 0xDE: # SBC n
                n = self.read_u8_imm()
                v = u8((self.AF >> 8) - n - self.C)
                self.Z = v == 0
                self.N = 1
                self.H = ((((self.AF >> 8) & 0xF) - (n & 0xF) - self.C) & 0x10) == 0x10
                self.C = (((self.AF >> 8) - n - self.C) & 0x100) == 0x100
                self.AF = (v << 8) | (self.AF & 0x00FF)

            case 0x10: # STOP
                print("TODO STOP")
                exit(1)

            case 0xAB: # XOR A, E
                self.AF = (((self.AF >> 8) ^ (self.DE & 0xFF)) << 8) | (self.AF & 0x00FF)
                self.Z = (self.AF >> 8) == 0
                self.N = 0
                self.H = 0
                self.C = 0

            case 0x51: # LD D, C
                self.DE = ((self.BC & 0xFF) << 8) | (self.DE & 0x00FF)

            case 0xE8: # ADD SP, i8
                e = i8(self.read_u8_imm())
                self.Z = 0
                self.N = 0
                self.H = (((self.SP & 0xF) + (e & 0xF)) & 0x10) == 0x10
                self.C = (((self.SP & 0xFF) + (e & 0xFF)) & 0x100) == 0x100
                self.SP = u16(self.SP + e)
                self.mem.tick() # internal cycle
                self.mem.tick() # SP write internal cycle (?)

            case 0xB3: # OR A, E
                self.AF |= ((self.DE & 0xFF) << 8)
                self.Z = (self.AF >> 8) == 0
                self.N = 0
                self.H = 0
                self.C = 0

            case 0x92: # SUB A, D
                v = u8((self.AF >> 8) - (self.DE >> 8))
                self.Z = v == 0
                self.N = 1
                self.H = ((((self.AF >> 8) & 0xF) - ((self.DE >> 8) & 0xF)) & 0x10) == 0x10
                self.C = (((self.AF >> 8) - (self.DE >> 8)) & 0x100) == 0x100
                self.AF = (v << 8) | (self.AF & 0x00FF)

            case 0x7F: # LD A, A
                pass

            case 0x84: # ADD A, H
                v = u8((self.AF >> 8) + (self.HL >> 8))
                self.Z = v == 0
                self.N = 0
                self.H = ((((self.AF >> 8) & 0xF) + ((self.HL >> 8) & 0xF)) & 0x10) == 0x10
                self.C = (((self.AF >> 8) + (self.HL >> 8)) & 0x100) == 0x100
                self.AF = (v << 8) | (self.AF & 0x00FF)
            
            case 0xF2: # LD A, (FF00+C)
                v = self.mem.read(0xFF00 | (self.BC & 0xFF))
                self.AF = (v << 8) | (self.AF & 0x00FF)

            case 0xD2: # JP NC, nn
                nn = self.read_u16_imm()
                if (not self.C):
                    self.PC = nn
                    self.mem.tick() # internal cycle

            case 0xA5: # AND A, L
                v = ((self.AF >> 8) & (self.HL & 0xFF))
                self.AF = (v << 8) | (self.AF & 0x00FF)
                self.Z = v == 0
                self.N = 0
                self.H = 1
                self.C = 0

            case 0x0A: # LD A, (BC)
                v = self.mem.read(self.BC)
                self.AF = (v << 8) | (self.AF & 0x00FF)

            case 0x5F: # LD E, A
                self.DE = (self.DE & 0xFF00) | (self.AF >> 8)

            case 0xBD: # CP A, L
                v = u8((self.AF >> 8) - (self.HL & 0xFF))
                self.Z = v == 0
                self.N = 1
                self.H = ((((self.AF >> 8) & 0xF) - ((self.HL & 0xFF) & 0xF)) & 0x10) == 0x10
                self.C = (((self.AF >> 8) - (self.HL & 0xFF)) & 0x100) == 0x100

            case 0x9E: # SBC A, (HL)
                n = self.mem.read(self.HL)
                v = u8((self.AF >> 8) - n - self.C)
                self.Z = v == 0
                self.N = 1
                self.H = ((((self.AF >> 8) & 0xF) - (n & 0xF) - self.C) & 0x10) == 0x10
                self.C = (((self.AF >> 8) - n - self.C) & 0x100) == 0x100
                self.AF = (v << 8) | (self.AF & 0x00FF)

            case 0x71: # LD (HL), C
                self.mem.write(self.HL, self.BC & 0xFF)

            case 0x8C: # ADC A, H
                v = u8((self.AF >> 8) + (self.HL >> 8) + self.C)
                self.Z = v == 0
                self.N = 0
                self.H = ((((self.AF >> 8) & 0xF) + ((self.HL >> 8) & 0xF) + self.C) & 0x10) == 0x10
                self.C = (((self.AF >> 8) + (self.HL >> 8) + self.C) & 0x100) == 0x100
                self.AF = (v << 8) | (self.AF & 0x00FF)

            case 0x88: # ADC A, B
                v = u8((self.AF >> 8) + (self.BC >> 8) + self.C)
                self.Z = v == 0
                self.N = 0
                self.H = ((((self.AF >> 8) & 0xF) + ((self.BC >> 8) & 0xF) + self.C) & 0x10) == 0x10
                self.C = (((self.AF >> 8) + (self.BC >> 8) + self.C) & 0x100) == 0x100
                self.AF = (v << 8) | (self.AF & 0x00FF)

            case 0x67: # LD H, A
                self.HL = ((self.AF >> 8) << 8) | (self.HL & 0x00FF)

            case 0x30: # JR NC, e
                e = i8(self.mem.read(self.PC))
                self.PC = u16(self.PC + 1)
                if (not self.C):
                    self.PC = u16(self.PC + e)
                    self.mem.tick() # internal cycle

            case 0xC8: # RET Z
                self.mem.tick() # internal cycle
                if self.Z:
                    lsb = self.mem.read(self.SP)
                    self.SP = u16(self.SP + 1)
                    msb = self.mem.read(self.SP)
                    self.SP = u16(self.SP + 1)
                    self.PC = (msb << 8) | lsb
                    self.mem.tick() # internal cycle
            
            case 0xCC: # CALL Z, nn
                nn = self.read_u16_imm()
                if (self.Z):
                    self.mem.tick() # internal cycle
                    self.SP = u16(self.SP - 1)
                    self.mem.write(self.SP, self.PC >> 8)
                    self.SP = u16(self.SP - 1)
                    self.mem.write(self.SP, self.PC & 0xFF)
                    self.PC = nn

            case 0x89: # ADC A, C
                v = u8((self.AF >> 8) + (self.BC & 0xFF) + self.C)
                self.Z = v == 0
                self.N = 0
                self.H = ((((self.AF >> 8) & 0xF) + (self.BC & 0xF) + self.C) & 0x10) == 0x10
                self.C = (((self.AF >> 8) + (self.BC & 0xFF) + self.C) & 0x100) == 0x100
                self.AF = (v << 8) | (self.AF & 0x00FF)

            case 0x66: # LD H,(HL)
                v = self.mem.read(self.HL)
                self.HL = (v << 8) | (self.HL & 0x00FF)
            
            case 0x8B: # ADC A, E
                v = u8((self.AF >> 8) + (self.DE & 0xFF) + self.C)
                self.Z = v == 0
                self.N = 0
                self.H = ((((self.AF >> 8) & 0xF) + (self.DE & 0xF) + self.C) & 0x10) == 0x10
                self.C = (((self.AF >> 8) + (self.DE & 0xFF) + self.C) & 0x100) == 0x100
                self.AF = (v << 8) | (self.AF & 0x00FF)

            case 0x70: # LD (HL), B
                self.mem.write(self.HL, self.BC >> 8)

            case 0x9D: # SBC A, L
                v = u8((self.AF >> 8) - (self.HL & 0xFF) - self.C)
                self.Z = v == 0
                self.N = 1
                self.H = ((((self.AF >> 8) & 0xF) - ((self.HL & 0xFF) & 0xF) - self.C) & 0x10) == 0x10
                self.C = (((self.AF >> 8) - (self.HL & 0xFF) - self.C) & 0x100) == 0x100
                self.AF = (v << 8) | (self.AF & 0x00FF)

            # https://blog.ollien.com/posts/gb-daa/
            # https://github.com/Baekalfen/PyBoy/blob/934054c385d8027a98185fbb8f23f34f20903adb/pyboy/core/opcodes.py#L422 (thank you!)
            case 0x27: # DAA
                v = self.AF >> 8
                corr = 0

                corr |= 0x06 if (self.H != 0) else 0x00
                corr |= 0x60 if (self.C != 0) else 0x00

                if (self.N) != 0:
                    v -= corr
                else:
                    corr |= 0x06 if (v & 0x0F) > 0x09 else 0x00
                    corr |= 0x60 if v > 0x99 else 0x00
                    v += corr

                self.AF = (u8(v) << 8) | (self.AF & 0x00FF)
                self.Z = (self.AF >> 8) == 0
                self.H = 0
                self.C = (corr & 0x60) != 0

            case 0xBE: # CP A, (HL)
                n = self.mem.read(self.HL)
                v = u8((self.AF >> 8) - n)
                self.Z = v == 0
                self.N = 1
                self.H = ((((self.AF >> 8) & 0xF) - (n & 0xF)) & 0x10) == 0x10
                self.C = (((self.AF >> 8) - n) & 0x100) == 0x100

            case 0xA4: # AND A, H
                v = ((self.AF >> 8) & (self.HL >> 8))
                self.AF = (v << 8) | (self.AF & 0x00FF)
                self.Z = v == 0
                self.N = 0
                self.H = 1
                self.C = 0

            case 0x1F: # RRA
                prev_c = self.C
                self.C = (self.AF >> 8) & 1
                self.AF = (((self.AF >> 9) | (prev_c << 7)) << 8) | (self.AF & 0x00FF)
                self.Z = 0
                self.N = 0
                self.H = 0

            case 0x4A: # LD C, D
                self.BC = (self.BC & 0xFF00) | (self.DE >> 8)

            case 0x3F: # CCF
                self.N = 0
                self.H = 0
                self.C = not self.C

            case 0x6A: # LD L, D
                self.HL = (self.HL & 0xFF00) | (self.DE >> 8)

            case 0x85: # ADD A, L
                v = u8((self.AF >> 8) + (self.HL & 0xFF))
                self.Z = v == 0
                self.N = 0
                self.H = ((((self.AF >> 8) & 0xF) + ((self.HL & 0xFF) & 0xF)) & 0x10) == 0x10
                self.C = (((self.AF >> 8) + (self.HL & 0xFF)) & 0x100) == 0x100
                self.AF = (v << 8) | (self.AF & 0x00FF)

            case 0x93: # SUB A, E
                v = u8((self.AF >> 8) - (self.DE & 0xFF))
                self.Z = v == 0
                self.N = 1
                self.H = ((((self.AF >> 8) & 0xF) - ((self.DE & 0xFF) & 0xF)) & 0x10) == 0x10
                self.C = (((self.AF >> 8) - (self.DE & 0xFF)) & 0x100) == 0x100
                self.AF = (v << 8) | (self.AF & 0x00FF)

            case 0xB2: # OR A, D
                self.AF |= ((self.DE >> 8) << 8)
                self.Z = (self.AF >> 8) == 0
                self.N = 0
                self.H = 0
                self.C = 0

            case 0x50: # LD D, B
                self.DE = ((self.BC >> 8) << 8) | (self.DE & 0x00FF)

            case 0xE9: # JP HL
                self.PC = self.HL

            case 0x07: # RLCA
                self.AF = (u8((self.AF >> 8) << 1) << 8) | ((self.AF >> 15) << 8) | (self.AF & 0x00FF)
                self.Z = 0
                self.N = 0
                self.H = 0
                self.C = (self.AF >> 8) & 1

            case 0xAC: # XOR A, H
                self.AF = (((self.AF >> 8) ^ (self.HL >> 8)) << 8) | (self.AF & 0x00FF)
                self.Z = (self.AF >> 8) == 0
                self.N = 0
                self.H = 0
                self.C = 0

            case 0xA8: # XOR A, B
                self.AF = (((self.AF >> 8) ^ (self.BC >> 8)) << 8) | (self.AF & 0x00FF)
                self.Z = (self.AF >> 8) == 0
                self.N = 0
                self.H = 0
                self.C = 0

            case 0xC7: # RST 00h
                self.mem.tick() # internal cycle
                self.SP = u16(self.SP - 1)
                self.mem.write(self.SP, self.PC >> 8)
                self.SP = u16(self.SP - 1)
                self.mem.write(self.SP, self.PC & 0xFF)
                self.PC = 0

            case 0x6B: # LD L, E
                self.HL = (self.HL & 0xFF00) | (self.DE & 0xFF)

            case 0x69: # LD L, C
                self.HL = (self.HL & 0xFF00) | (self.BC & 0xFF)

            case 0x86: # ADD A, (HL)
                n = self.mem.read(self.HL)
                v = u8((self.AF >> 8) + n)
                self.Z = v == 0
                self.N = 0
                self.H = ((((self.AF >> 8) & 0xF) + (n & 0xF)) & 0x10) == 0x10
                self.C = (((self.AF >> 8) + n) & 0x100) == 0x100
                self.AF = (v << 8) | (self.AF & 0x00FF)

            case 0x90: # SUB A, B
                v = u8((self.AF >> 8) - (self.BC >> 8))
                self.Z = v == 0
                self.N = 1
                self.H = ((((self.AF >> 8) & 0xF) - ((self.BC >> 8) & 0xF)) & 0x10) == 0x10
                self.C = (((self.AF >> 8) - (self.BC >> 8)) & 0x100) == 0x100
                self.AF = (v << 8) | (self.AF & 0x00FF)

            case 0x53: # LD D, E
                self.DE = ((self.DE & 0xFF) << 8) | (self.DE & 0x00FF)

            case 0x04: # INC B
                v = u8((self.BC >> 8) + 1)
                self.Z = v == 0
                self.N = 0
                self.H = ((((self.BC >> 8) & 0xF) + 1) & 0x10) == 0x10
                self.BC = (v << 8) | (self.BC & 0x00FF)

            case 0x12: # LD (DE), A
                self.mem.write(self.DE, self.AF >> 8)

            case 0x45: # LD B, L
                self.BC = ((self.HL & 0xFF) << 8) | (self.BC & 0x00FF)

            case 0x65: # LD H, L
                self.HL = ((self.HL & 0xFF) << 8) | (self.HL & 0x00FF)

            case 0x8A: # ADC A, D
                v = u8((self.AF >> 8) + (self.DE >> 8) + self.C)
                self.Z = v == 0
                self.N = 0
                self.H = ((((self.AF >> 8) & 0xF) + ((self.DE >> 8) & 0xF) + self.C) & 0x10) == 0x10
                self.C = (((self.AF >> 8) + (self.DE >> 8) + self.C) & 0x100) == 0x100
                self.AF = (v << 8) | (self.AF & 0x00FF)

            case 0x73: # LD (HL), E
                self.mem.write(self.HL, self.DE & 0xFF)

            case 0xBF: # CP A, A
                self.Z = 1
                self.N = 1
                self.H = 0
                self.C = 0

            case 0x5D: # LD E, L
                self.DE = (self.DE & 0xFF00) | (self.HL & 0xFF)

            case 0x08: # LD (nn), SP
                nn = self.read_u16_imm()
                self.mem.write(nn, self.SP & 0xFF)
                self.mem.write(nn + 1, self.SP >> 8)

            case 0x06: # LD B, n
                n = self.read_u8_imm()
                self.BC = (n << 8) | (self.BC & 0x00FF)

            case 0x0C: # INC C
                v = u8((self.BC & 0xFF) + 1)
                self.Z = v == 0
                self.N = 0
                self.H = ((((self.BC & 0xFF) & 0xF) + 1) & 0x10) == 0x10
                self.BC = (self.BC & 0xFF00) | v

            case 0xA7: # AND A, A
                self.Z = (self.AF >> 8) == 0
                self.N = 0
                self.H = 1
                self.C = 0

            case 0x1E: # LD E, n
                n = self.read_u8_imm()
                self.DE = (self.DE & 0xFF00) | n

            case 0xD0: # RET NC
                self.mem.tick() # internal cycle
                if not self.C:
                    lsb = self.mem.read(self.SP)
                    self.SP = u16(self.SP + 1)
                    msb = self.mem.read(self.SP)
                    self.SP = u16(self.SP + 1)
                    self.PC = (msb << 8) | lsb
                    self.mem.tick() # internal cycle

            case 0x49: # LD C, C
                pass

            case 0x4B: # LD C, E
                self.BC = (self.BC & 0xFF00) | (self.DE & 0xFF)

            case 0x4C: # LD C, H
                self.BC = (self.BC & 0xFF00) | (self.HL >> 8)

            case 0xD1: # POP DE
                lsb = self.mem.read(self.SP)
                self.SP = u16(self.SP + 1)
                msb = self.mem.read(self.SP)
                self.SP = u16(self.SP + 1)
                self.DE = (msb << 8) | lsb

            case 0x48: # LD C, B
                self.BC = (self.BC & 0xFF00) | (self.BC >> 8)

            case 0x1D: # DEC E
                v = u8((self.DE & 0xFF) - 1)
                self.Z = v == 0
                self.N = 1
                self.H = (((self.DE & 0xF) - (1 & 0xF)) & 0x10) == 0x10
                self.DE = (self.DE & 0xFF00) | v

            case 0xA6: # AND A, (HL)
                n = self.mem.read(self.HL)
                v = ((self.AF >> 8) & n)
                self.AF = (v << 8) | (self.AF & 0x00FF)
                self.Z = v == 0
                self.N = 0
                self.H = 1
                self.C = 0

            case 0x0B: # DEC BC
                self.BC = u16(self.BC - 1)
                self.mem.tick() # internal cycle

            case 0x09: # ADD HL, BC
                self.N = 0
                self.H = (((self.HL & 0xFFF) + (self.BC & 0xFFF)) & 0x1000) == 0x1000
                self.C = ((self.HL + self.BC) & 0x10000) == 0x10000
                self.HL = u16(self.HL + self.BC)
                self.mem.tick() # internal cycle

            case 0x5E: # LD E, (HL)
                v = self.mem.read(self.HL)
                self.DE = (self.DE & 0xFF00) | v

            case 0xE7: # RST 20h
                self.mem.tick() # internal cycle
                self.SP = u16(self.SP - 1)
                self.mem.write(self.SP, self.PC >> 8)
                self.SP = u16(self.SP - 1)
                self.mem.write(self.SP, self.PC & 0xFF)
                self.PC = 0x20

            case 0x25: # DEC H
                v = u8((self.HL >> 8) - 1)
                self.Z = v == 0
                self.N = 1
                self.H = ((((self.HL >> 8) & 0xF) - (1 & 0xF)) & 0x10) == 0x10
                self.HL = (v << 8) | (self.HL & 0x00FF)

            case 0x9F: # SBC A, A
                v = u8((self.AF >> 8) - (self.AF >> 8) - self.C)
                self.Z = v == 0
                self.N = 1
                self.H = ((((self.AF >> 8) & 0xF) - ((self.AF >> 8) & 0xF) - self.C) & 0x10) == 0x10
                self.C = (((self.AF >> 8) - (self.AF >> 8) - self.C) & 0x100) == 0x100
                self.AF = (v << 8) | (self.AF & 0x00FF)

            case 0x72: # LD (HL), D
                self.mem.write(self.HL, self.DE >> 8)

            case 0xFF: # RST 38h 
                self.mem.tick() # internal cycle
                self.SP = u16(self.SP - 1)
                self.mem.write(self.SP, self.PC >> 8)
                self.SP = u16(self.SP - 1)
                self.mem.write(self.SP, self.PC & 0xFF)
                self.PC = 0x38

            case 0x64: # LD H, H
                pass

            case 0xCA: # JP Z, nn
                nn = self.read_u16_imm()
                if (self.Z):
                    self.PC = nn
                    self.mem.tick() # internal cycle

            case 0x33: # INC SP
                self.SP = u16(self.SP + 1)
                self.mem.tick() # internal cycle

            case 0x44: # LD B, H
                self.BC = ((self.HL >> 8) << 8) | (self.BC & 0x00FF)

            case 0xDF: # RST 18h
                self.mem.tick() # internal cycle
                self.SP = u16(self.SP - 1)
                self.mem.write(self.SP, self.PC >> 8)
                self.SP = u16(self.SP - 1)
                self.mem.write(self.SP, self.PC & 0xFF)
                self.PC = 0x18

            case 0xAA: # XOR A, D
                self.AF = (((self.AF >> 8) ^ (self.DE >> 8)) << 8) | (self.AF & 0x00FF)
                self.Z = (self.AF >> 8) == 0
                self.N = 0
                self.H = 0
                self.C = 0

            case 0x52: # LD D, D
                pass

            case 0x29: # ADD HL, HL
                self.N = 0
                self.H = (((self.HL & 0xFFF) + (self.HL & 0xFFF)) & 0x1000) == 0x1000
                self.C = ((self.HL + self.HL) & 0x10000) == 0x10000
                self.HL = u16(self.HL + self.HL)
                self.mem.tick() # internal cycle

            case 0xB0: # OR A, B
                self.AF |= ((self.BC >> 8) << 8)
                self.Z = (self.AF >> 8) == 0
                self.N = 0
                self.H = 0
                self.C = 0

            case 0x2B: # DEC HL
                self.HL = u16(self.HL - 1)
                self.mem.tick()

            case 0x91: # SUB A, C
                v = u8((self.AF >> 8) - (self.BC & 0xFF))
                self.Z = v == 0
                self.N = 1
                self.H = ((((self.AF >> 8) & 0xF) - ((self.BC & 0xFF) & 0xF)) & 0x10) == 0x10
                self.C = (((self.AF >> 8) - (self.BC & 0xFF)) & 0x100) == 0x100
                self.AF = (v << 8) | (self.AF & 0x00FF)

            case 0x7E: # LD A, (HL)
                v = self.mem.read(self.HL)
                self.AF = (v << 8) | (self.AF & 0x00FF)

            case 0x68: # LD L, B
                self.HL = (self.HL & 0xFF00) | (self.BC >> 8)

            case 0x87: # ADD A, A
                v = u8((self.AF >> 8) + (self.AF >> 8))
                self.Z = v == 0
                self.N = 0
                self.H = ((((self.AF >> 8) & 0xF) + ((self.AF >> 8) & 0xF)) & 0x10) == 0x10
                self.C = (((self.AF >> 8) + (self.AF >> 8)) & 0x100) == 0x100
                self.AF = (v << 8) | (self.AF & 0x00FF)

            case 0x6C: # LD L, H
                self.HL = (self.HL & 0xFF00) | (self.HL >> 8)

            case 0x3D: # DEC A
                v = u8((self.AF >> 8) - 1)
                self.Z = v == 0
                self.N = 1
                self.H = ((((self.AF >> 8) & 0xF) - (1 & 0xF)) & 0x10) == 0x10
                self.AF = (v << 8) | (self.AF & 0x00FF)

            case 0x34: # INC (HL)
                n = self.mem.read(self.HL)
                v = u8(n + 1)
                self.Z = v == 0
                self.N = 0
                self.H = (((n & 0xF) + 1) & 0x10) == 0x10
                self.mem.write(self.HL, v)

            case 0xCF: # RST 08h
                self.mem.tick() # internal cycle
                self.SP = u16(self.SP - 1)
                self.mem.write(self.SP, self.PC >> 8)
                self.SP = u16(self.SP - 1)
                self.mem.write(self.SP, self.PC & 0xFF)
                self.PC = 0x08

            case 0x63: # LD H, E
                self.HL = ((self.DE & 0xFF) << 8) | (self.HL & 0x00FF)

            case 0x75: # LD (HL), L
                self.mem.write(self.HL, self.HL & 0xFF)

            case 0x9A: # SBC A, D
                v = u8((self.AF >> 8) - (self.DE >> 8) - self.C)
                self.Z = v == 0
                self.N = 1
                self.H = ((((self.AF >> 8) & 0xF) - ((self.DE >> 8) & 0xF) - self.C) & 0x10) == 0x10
                self.C = (((self.AF >> 8) - (self.DE >> 8) - self.C) & 0x100) == 0x100
                self.AF = (v << 8) | (self.AF & 0x00FF)

            case 0x59: # LD E, C
                self.DE = (self.DE & 0xFF00) | (self.BC & 0xFF)

            case 0x5B: # LD E, E
                pass

            case 0xA1: # AND A, C
                v = ((self.AF >> 8) & (self.BC & 0xFF))
                self.AF = (v << 8) | (self.AF & 0x00FF)
                self.Z = v == 0
                self.N = 0
                self.H = 1
                self.C = 0

            case 0x1C: # INC E
                v = u8((self.DE & 0xFF) + 1)
                self.Z = v == 0
                self.N = 0
                self.H = (((self.DE & 0xF) + 1) & 0x10) == 0x10
                self.DE = (self.DE & 0xFF00) | v

            case 0x4D: # LD C, L
                self.BC = (self.BC & 0xFF00) | (self.HL & 0xFF)

            case 0x3C: # INC A
                v = u8((self.AF >> 8) + 1)
                self.Z = v == 0
                self.N = 0
                self.H = ((((self.AF >> 8) & 0xF) + 1) & 0x10) == 0x10
                self.AF = (v << 8) | (self.AF & 0x00FF)

            case 0x38: # JR C, e
                e = i8(self.mem.read(self.PC))
                self.PC = u16(self.PC + 1)
                if (self.C):
                    self.PC = u16(self.PC + e)
                    self.mem.tick() # internal cycle

            case 0x6D: # LD L, L
                pass

            case 0x80: # ADD A, B
                v = u8((self.AF >> 8) + (self.BC >> 8))
                self.Z = v == 0
                self.N = 0
                self.H = ((((self.AF >> 8) & 0xF) + ((self.BC >> 8) & 0xF)) & 0x10) == 0x10
                self.C = (((self.AF >> 8) + (self.BC >> 8)) & 0x100) == 0x100
                self.AF = (v << 8) | (self.AF & 0x00FF)

            case 0xF6: # OR A, n
                n = self.read_u8_imm()
                self.AF |= (n << 8)
                self.Z = (self.AF >> 8) == 0
                self.N = 0
                self.H = 0
                self.C = 0

            case 0x7B: # LD A, E
                self.AF = ((self.DE & 0xFF) << 8) | (self.AF & 0x00FF)

            case 0x79: # LD A, C
                self.AF = ((self.BC & 0xFF) << 8) | (self.AF & 0x00FF)

            case 0x96: # SUB A, (HL)
                n = self.mem.read(self.HL)
                v = u8((self.AF >> 8) - n)
                self.Z = v == 0
                self.N = 1
                self.H = ((((self.AF >> 8) & 0xF) - (n & 0xF)) & 0x10) == 0x10
                self.C = (((self.AF >> 8) - n) & 0x100) == 0x100
                self.AF = (v << 8) | (self.AF & 0x00FF)

            case 0x2E: # LD L, n
                n = self.read_u8_imm()
                self.HL = (self.HL & 0xFF00) | n

            case 0x55: # LD D, L
                self.DE = ((self.HL & 0xFF) << 8) | (self.DE & 0x00FF)

            case 0xAF: # XOR A, A
                self.AF = self.AF & 0x00FF
                self.Z = 1
                self.N = 0
                self.H = 0
                self.C = 0

            case 0x14: # INC D
                v = u8((self.DE >> 8) + 1)
                self.Z = v == 0
                self.N = 0
                self.H = ((((self.DE >> 8) & 0xF) + 1) & 0x10) == 0x10
                self.DE = (v << 8) | (self.DE & 0x00FF)

            case 0xDA: # JP C, nn
                nn = self.read_u16_imm()
                if (self.C):
                    self.PC = nn
                    self.mem.tick() # internal cycle

            case 0x43: # LD B, E
                self.BC = ((self.DE & 0xFF) << 8) | (self.BC & 0x00FF)

            case 0x42: # LD B, D
                self.BC = ((self.DE >> 8) << 8) | (self.BC & 0x00FF)

            case 0x15: # DEC D
                v = u8((self.DE >> 8) - 1)
                self.Z = v == 0
                self.N = 1
                self.H = ((((self.DE >> 8) & 0xF) - (1 & 0xF)) & 0x10) == 0x10
                self.DE = (v << 8) | (self.DE & 0x00FF)

            case 0x54: # LD D, H
                self.DE = ((self.HL >> 8) << 8) | (self.DE & 0x00FF)

            case 0xEF: # RST 28h
                self.mem.tick() # internal cycle
                self.SP = u16(self.SP - 1)
                self.mem.write(self.SP, self.PC >> 8)
                self.SP = u16(self.SP - 1)
                self.mem.write(self.SP, self.PC & 0xFF)
                self.PC = 0x28

            case 0xB6: # OR A, (HL)
                v = self.mem.read(self.HL)
                self.AF |= (v << 8)
                self.Z = (self.AF >> 8) == 0
                self.N = 0
                self.H = 0
                self.C = 0

            case 0x97: # SUB A, A
                self.AF = self.AF & 0x00FF
                self.Z = 1
                self.N = 1
                self.H = 0
                self.C = 0

            case 0x81: # ADD A, C
                v = u8((self.AF >> 8) + (self.BC & 0xFF))
                self.Z = v == 0
                self.N = 0
                self.H = ((((self.AF >> 8) & 0xF) + ((self.BC & 0xFF) & 0xF)) & 0x10) == 0x10
                self.C = (((self.AF >> 8) + (self.BC & 0xFF)) & 0x100) == 0x100
                self.AF = (v << 8) | (self.AF & 0x00FF)

            case 0xF7: # RST 30h
                self.mem.tick() # internal cycle
                self.SP = u16(self.SP - 1)
                self.mem.write(self.SP, self.PC >> 8)
                self.SP = u16(self.SP - 1)
                self.mem.write(self.SP, self.PC & 0xFF)
                self.PC = 0x30

            case 0x6E: # LD L, (HL)
                v = self.mem.read(self.HL)
                self.HL = (self.HL & 0xFF00) | v

            case 0x39: # ADD HL, SP
                self.N = 0
                self.H = (((self.HL & 0xFFF) + (self.SP & 0xFFF)) & 0x1000) == 0x1000
                self.C = ((self.HL + self.SP) & 0x10000) == 0x10000
                self.HL = u16(self.HL + self.SP)
                self.mem.tick() # internal cycle

            case 0xC0: # RET NZ
                self.mem.tick() # internal cycle
                if not self.Z:
                    lsb = self.mem.read(self.SP)
                    self.SP = u16(self.SP + 1)
                    msb = self.mem.read(self.SP)
                    self.SP = u16(self.SP + 1)
                    self.PC = (msb << 8) | lsb
                    self.mem.tick() # internal cycle

            case 0x3B: # DEC SP 
                self.SP = u16(self.SP - 1)
                self.mem.tick()

            case 0xD7: # RST 10h
                self.mem.tick() # internal cycle
                self.SP = u16(self.SP - 1)
                self.mem.write(self.SP, self.PC >> 8)
                self.SP = u16(self.SP - 1)
                self.mem.write(self.SP, self.PC & 0xFF)
                self.PC = 0x10

            case 0x1B: # DEC DE
                self.DE = u16(self.DE - 1)
                self.mem.tick()

            case 0xA0: # AND A, B
                v = ((self.AF >> 8) & (self.BC >> 8))
                self.AF = (v << 8) | (self.AF & 0x00FF)
                self.Z = v == 0
                self.N = 0
                self.H = 1
                self.C = 0

            case 0x19: # ADD HL, DE
                self.N = 0
                self.H = (((self.HL & 0xFFF) + (self.DE & 0xFFF)) & 0x1000) == 0x1000
                self.C = ((self.HL + self.DE) & 0x10000) == 0x10000
                self.HL = u16(self.HL + self.DE)
                self.mem.tick() # internal cycle

            case 0x0D: # DEC C
                v = u8((self.BC & 0xFF) - 1)
                self.Z = v == 0
                self.N = 1
                self.H = (((self.BC & 0xF) - 1) & 0x10) == 0x10
                self.BC = (self.BC & 0xFF00) | v

            case 0x5C: # LD E, H
                self.DE = (self.DE & 0xFF00) | (self.HL >> 8)

            case 0x58: # LD E, B
                self.DE = (self.DE & 0xFF00) | (self.BC >> 8)

            case 0xBA: # CP A, D
                v = u8((self.AF >> 8) - (self.DE >> 8))
                self.Z = v == 0
                self.N = 1
                self.H = ((((self.AF >> 8) & 0xF) - ((self.DE >> 8) & 0xF)) & 0x10) == 0x10
                self.C = (((self.AF >> 8) - (self.DE >> 8)) & 0x100) == 0x100

            case 0x74: # LD (HL), H
                self.mem.write(self.HL, self.HL >> 8)

            case 0x8F: # ADC A, A
                v = u8((self.AF >> 8) + (self.AF >> 8) + self.C)
                self.Z = v == 0
                self.N = 0
                self.H = ((((self.AF >> 8) & 0xF) + ((self.AF >> 8) & 0xF) + self.C) & 0x10) == 0x10
                self.C = (((self.AF >> 8) + (self.AF >> 8) + self.C) & 0x100) == 0x100
                self.AF = (v << 8) | (self.AF & 0x00FF)

            case 0x62: # LD H, D
                self.HL = ((self.DE >> 8) << 8) | (self.HL & 0x00FF)

            case 0x35: # DEC (HL)
                n = self.mem.read(self.HL)
                v = u8(n - 1)
                self.Z = v == 0
                self.N = 1
                self.H = (((n & 0xF) - 1) & 0x10) == 0x10
                self.mem.write(self.HL, v)

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
            if self.delayed_ime_enable:
                self.ime = True
                self.delayed_ime_enable = False

            self.execute()

            # after executing each instruction, the number of M-cycles that passed
            # during the instruction will be tracked in memory and applied here.
            # number of ticks in memory should be reset at the end of each instruction 
            m_cycles_ran += self.mem.ticks_per_instr
            self.mem.ticks_per_instr = 0

        return self.mem.ppu.frame
