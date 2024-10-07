class PPU:
    def __init__(self):
        self.vram = [0] * 0x2000 # 0x8000 - 0x9FFF
        self.oam = [0] * 0xA0 # 0xFE00 - 0xFE9F

        self.LY = 0
        self.LCDC = 0x0
        self.SCX = 0x0
        self.SCY = 0x0
        self.BGP = 0x0
        self.frame = [[0] * 160] * 144

    def tick(self):
        pass