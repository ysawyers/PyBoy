class Memory:
    def __init__(self, rom: str):
        self.rom = []
        self.vram = [0] * 0x2000 # 0x8000 - 0x9FFF (8 KiB)
        self.eram = [0] * 0x2000 # 0xA000 - 0xBFFF (8 KiB)
        self.wram = [0] * 0x2000 # 0xC000 - 0xDFFF (8 KiB)
        # 0xE000 - 0xFDFF (PROHIBITED BY NINTENDO)
        self.oam = [0] * 0xA0
        # 0xFEA0 - 0xFEFF (PROHIBITED BY NINTENDO)
        self.mmio = [0] * 0x80
        self.hram = [0] * 0x7F

        with open(rom, "rb") as f:
            self.rom = f.read()

    def read(self, addr: int):
        pass

    def write(self, addr: int, value: int):
        pass

class CPU:
    def __init__(self, rom: str):
        # 16-bit registers that are made up of two smaller 8-bit registers
        # ex. AF = A(upper 8 bit register) + F(lower 8 bit register) = 16-bit register
        self.AF = 0
        self.BC = 0
        self.DE = 0
        self.HL = 0

        # strictly 16-bit only
        self.SP = 0 # stack pointer
        self.PC = 0 # program counter

        # CPU Flags (used to perform conditional branching AKA if statements)
        # NOTE: each represent 1 bit and together are the upper 4 bits of the F register
        self.Z = 0 # zero flag
        self.N = 0 # negative (AKA substract) flag
        self.H = 0 # half-carry flag
        self.C = 0 # carry flag

        self.mem = Memory(rom)

    def execute(self) -> int:
        # each instruction starts by reading the byte at PC which represents the opcode
        # (hint: this is where you reference an opcode table to figure out how to process it)
        opcode = self.mem.rom[self.PC]
        self.PC += 1

        match opcode:
            case _:
                print(hex(opcode))
                exit(1)
        

    def render_frame(self):
        for i in range(10):
            self.execute()

cpu = CPU("cpu_instrs.gb")
cpu.render_frame()