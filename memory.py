class Memory:
    def __init__(self, rom: str):
        self.rom: [int] = []
        self.vram = [0] * 0x2000 # 0x8000 - 0x9FFF (8 KiB)
        self.eram = [0] * 0x2000 # 0xA000 - 0xBFFF (8 KiB)
        self.wram = [0] * 0x2000 # 0xC000 - 0xDFFF (8 KiB)
        # 0xE000 - 0xFDFF (PROHIBITED BY NINTENDO)
        self.oam = [0] * 0xA0
        # 0xFEA0 - 0xFEFF (PROHIBITED BY NINTENDO)
        self.mmio = [0] * 0x80
        self.hram = [0] * 0x7F

        # loading the game rom into our emulator
        with open(rom, "rb") as f:
            self.rom = f.read()

    def read(self, addr: int):
        pass

    def write(self, addr: int, value: int):
        pass