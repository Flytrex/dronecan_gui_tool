#
# Copyright (C) 2026  UAVCAN Development Team  <dronecan.org>
#
# This software is distributed under the terms of the MIT License.
#
# Author: Ilan Graidy
# Date:   2026-03-10
#

"""
Utility functions for DroneCAN GUI Tool panels.
"""

def _crc_step(input_data: int, poly: int = 0x04C11DB7, initial_crc: int = 0x464C5458, sizeof_input: int = 32) -> int:
    """ Adapted from https://github.com/HeMe2/stm32_crc_tool/blob/master/stm32_crc_tool.py
        This is a direct emulation of one step of the STM32G4's CRC peripheral """

    def msb(val: int, bits: int = 8) -> bool:
        return bool(val & (1 << (bits - 1)))

    # create the proper mask
    msk = 0
    for i in range(sizeof_input):
        msk = (msk << 1) | 1

    # start of the algorithm described in the stm32 crc application manual
    crc = initial_crc ^ input_data

    b_index = 0
    while b_index < sizeof_input:
        if msb(crc, sizeof_input):
            crc = ((crc << 1) ^ poly) & msk
        else:
            crc = (crc << 1) & msk
        b_index += 1

    return crc

def crc32_stm32_batch(input_data: bytes, initial_crc: int = 0x464C5458):
    """ This matches the CRC settings used in the firmware """
    assert len(input_data) % 4 == 0
    crc32 = initial_crc
    n_ints = len(input_data) // 4
    for i in range(n_ints):
        next_num = int.from_bytes(input_data[(4 * i):(4 * (i + 1))], byteorder='little', signed=False)
        crc32 = _crc_step(next_num, initial_crc=crc32)
    return crc32

