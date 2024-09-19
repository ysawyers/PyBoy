class Memory:
    def __init__(self, rom: str):
        self.rom: [int] = []

        # loading the "game cartridge" into our emulator
        with open(rom, "rb") as f:
            self.rom = f.read()

    # sends data from memory to our CPU
    def read(self, addr: int) -> int:
        print("attempted to read to:", hex(addr))
        exit(1)

    # sends data from our CPU to memory
    def write(self, addr: int, value: int):
        print("attempted to write to:", hex(addr), "with value:", hex(value))
        exit(1)






























'''
IGNORE FOR NOW:
self.vram = [0] * 0x2000 # 0x8000 - 0x9FFF (8 KiB)
self.eram = [0] * 0x2000 # 0xA000 - 0xBFFF (8 KiB)
self.wram = [0] * 0x2000 # 0xC000 - 0xDFFF (8 KiB)
# 0xE000 - 0xFDFF (PROHIBITED BY NINTENDO)
self.oam = [0] * 0xA0
# 0xFEA0 - 0xFEFF (PROHIBITED BY NINTENDO)
self.mmio = [0] * 0x80
self.hram = [0] * 0x7F

# loading the boot rom into our emulator
        with open("bootrom.bin", "rb") as f:
            self.bootrom = f.read()

REMEMBER TO ALSO SET LCDC TO 0x91
'''