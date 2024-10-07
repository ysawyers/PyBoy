import unittest
import json
from os import listdir
from cpu import CPU

class TestCPUOps(unittest.TestCase):
    def initialize_registers(self, cpu: CPU, initial):
        cpu.PC = initial["pc"]
        cpu.SP = initial["sp"]
        cpu.AF = (initial["a"] << 8) | (initial["f"] & 0x0F)
        cpu.BC = (initial["b"] << 8) | (initial["c"] & 0xFF)
        cpu.DE = (initial["d"] << 8) | (initial["e"] & 0xFF)
        cpu.HL = (initial["h"] << 8) | (initial["l"] & 0xFF)

        cpu.Z = (initial["f"] >> 7) & 1
        cpu.N = (initial["f"] >> 6) & 1
        cpu.H = (initial["f"] >> 5) & 1
        cpu.C = (initial["f"] >> 4) & 1

        for item in initial["ram"]:
            cpu.mem.memory[item[0]] = item[1]

    def test_jsmooSM83(self):
        cpu = CPU(None)

        for test_filename in listdir("jsmoo"):
            with open(f'jsmoo/{test_filename}') as json_file:
                opcode_tests = json.load(json_file)
                for test in opcode_tests:
                    self.initialize_registers(cpu, test["initial"])
                    cpu.execute()

                    final = test["final"]

                    self.assertEqual(final["a"], cpu.AF >> 8)
                    self.assertEqual(final["f"], cpu.flag_bits() | (cpu.AF & 0x0F))
                    self.assertEqual(final["b"], cpu.BC >> 8)
                    self.assertEqual(final["c"], cpu.BC & 0xFF)
                    self.assertEqual(final["d"], cpu.DE >> 8)
                    self.assertEqual(final["e"], cpu.DE & 0xFF)
                    self.assertEqual(final["h"], cpu.HL >> 8)
                    self.assertEqual(final["l"], cpu.HL & 0xFF)
                    self.assertEqual(final["pc"], cpu.PC)
                    self.assertEqual(final["sp"], cpu.SP)
                    
                    for item in final["ram"]:
                        self.assertEqual(cpu.mem.memory[item[0]], item[1])

                    self.assertEqual(len(test["cycles"]), cpu.mem.ticks_per_instr)
                    cpu.mem.ticks_per_instr = 0

unittest.main()