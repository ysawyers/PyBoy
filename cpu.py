from memory import Memory

class CPU:
    M_CYCLES_PER_FRAME = 17556

    def __init__(self, rom: str):
        # 16-bit registers that are made up of two smaller 8-bit registers
        # ex. AF = A(upper 8 bit register) + F(lower 8 bit register) = 16-bit register
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

    def execute(self) -> int:
        # each instruction starts by reading the byte at PC which represents the opcode
        # (hint: this is where you reference an opcode table to figure out how to process it)
        opcode = self.mem.read(self.PC)
        self.PC += 1

        match opcode:
            case _:
                print(hex(opcode))
                exit(1)

    def render_frame(self) -> [[int]]:
        # each frame takes a fixed length of "time" to render and the way
        # we represent this "time" is through CPU M-cycles. An instruction
        # that is executed can vary between 1-N M-cycles (machine cycles) and
        # we sum the total amount of M-cycles processed per frame here
        m_cycles_ran = 0

        # After 17556 M-cycles, a frame is rendered!
        while m_cycles_ran < self.M_CYCLES_PER_FRAME:
            m_cycles_ran += self.execute()

        return []