#
# Copyright (C) 2016  UAVCAN Development Team  <uavcan.org>
#
# This software is distributed under the terms of the MIT License.
#
# Author: Pavel Kirienko <pavel.kirienko@zubax.com>
#

import dronecan
from dronecan.transport import Transfer, Frame

# How many rows will be traversed while looking for beginning/end of a multi frame transfer
TABLE_TRAVERSING_RANGE = 2000


class DecodingFailedException(Exception):
    pass


def _get_transfer_id(frame):
    if len(frame.data):
        return frame.data[-1] & 0b00011111


def _is_start_of_transfer(frame):
    if len(frame.data):
        return frame.data[-1] & 0b10000000


def _is_end_of_transfer(frame):
    if len(frame.data):
        return frame.data[-1] & 0b01000000


def _frame_has_tail_byte(frame):
    return bool(getattr(frame, 'data', None)) and len(frame.data) > 0


def _frame_payload_without_tail(frame):
    if not _frame_has_tail_byte(frame):
        return b''
    return bytes(frame.data[:-1])


def get_payload_from_transfer(transfer, frames=None):
    """
    Inspect the recovered transfer to determine its DSDL type and
    render the payload fields in a readable form.

    Uses the decoded, typed payload constructed by Transfer.from_frames().
    """
    try:
        dtype = dronecan.get_dronecan_data_type(transfer.payload)
        type_name = dtype.full_name if dtype else '<unknown>'
    except Exception:
        dtype = None
        type_name = '<unknown>'

    kind = 'service' if transfer.service_not_message else 'message'
    mode = None
    if transfer.service_not_message:
        mode = 'request' if transfer.request_not_response else 'response'

    # Anonymous applies to message frames with source_node_id == 0
    is_anonymous = (not transfer.service_not_message) and (transfer.source_node_id == 0)

    header = [
        f"Type: {type_name}",
        f"Frame Type: {kind}",
        f"Data Type ID: {transfer.data_type_id}",
        f"Priority: {transfer.transfer_priority}",
    ]
    if mode:
        header.append(f"Mode: {mode}")
    header.append(f"Anonymous: {'yes' if is_anonymous else 'no'}")
    if is_anonymous and transfer.discriminator is not None:
        header.append(f"Discriminator: {transfer.discriminator}")
    # Tail byte diagnostics and payload bytes if frames provided
    if frames:
        try:
            def fmt_tail(tb):
                tid = tb & 0x1F
                tog = 1 if (tb & 0x20) else 0
                eot = 1 if (tb & 0x40) else 0
                sot = 1 if (tb & 0x80) else 0
                return f"0x{tb:02X} [Start of Transfer={sot} End of Transfer={eot} Toggle={tog} Transfer ID={tid}]"

            frames_with_tail = [f for f in frames if _frame_has_tail_byte(f)]
            first_tail = frames_with_tail[0].data[-1] if frames_with_tail else None
            last_tail = frames_with_tail[-1].data[-1] if frames_with_tail else None
            header.append(f"Frames: {len(frames)}")
            if len(frames_with_tail) != len(frames):
                header.append(f"Frames without tail byte: {len(frames) - len(frames_with_tail)}")
            if first_tail is not None:
                header.append(f"First tail: {fmt_tail(first_tail)}")
            if last_tail is not None and (len(frames_with_tail) > 1 or last_tail != first_tail):
                header.append(f"Last tail:  {fmt_tail(last_tail)}")

            # Payload bytes (per frame) and reconstructed payload
            header.append("Payload bytes (per frame):")
            reconstructed = bytearray()
            for idx, f in enumerate(frames):
                part = _frame_payload_without_tail(f)
                reconstructed += part
                hex_part = ' '.join(f"{b:02X}" for b in part)
                header.append(f"  F{idx}: {len(part)} bytes: {hex_part}")
            header.append(f"Reconstructed payload: {len(reconstructed)} bytes")
            if reconstructed:
                hex_full = ' '.join(f"{b:02X}" for b in reconstructed)
                header.append(f"Payload hex: {hex_full}")

            # CRC reporting for multi-frame transfers
            if len(frames) > 1:
                payload_bytes = bytearray(b''.join(_frame_payload_without_tail(f) for f in frames))
                if len(payload_bytes) >= 2:
                    transfer_crc = payload_bytes[0] | (payload_bytes[1] << 8)
                    header.append(f"Transfer CRC (from frames): 0x{transfer_crc:04X}")
                    try:
                        dtype = dronecan.get_dronecan_data_type(transfer.payload)
                        base_crc = getattr(dtype, 'base_crc', None)
                        if base_crc is not None:
                            # In UAVCAN v0 multi-frame transfers, the first two payload bytes contain
                            # the transfer CRC; the CRC is computed over the remaining bytes.
                            computed = dronecan.dsdl.common.crc16_from_bytes(payload_bytes[2:], initial=base_crc)
                            header.append(f"Computed CRC:               0x{computed:04X} (base 0x{base_crc:04X})")
                            header.append(f"CRC match:                  {'yes' if computed == transfer_crc else 'no'}")
                    except Exception:
                        pass
        except Exception:
            # Reporting is best-effort; ignore failures
            pass

    payload_section_header  = f"\nParsed payload:"
    yaml_dump = dronecan.to_yaml(transfer.payload)

    parts = ['\n'.join(header), payload_section_header , yaml_dump]
    return "\n".join(parts)


def decode_transfer_from_frame(entry_row, row_to_frame):
    entry_frame, direction = row_to_frame(entry_row)
    can_id = entry_frame.id
    transfer_id = _get_transfer_id(entry_frame)
    frames = [entry_frame]

    related_rows = []

    # Scanning backward looking for the first frame
    row = entry_row - 1
    while not _is_start_of_transfer(frames[0]):
        if row < 0 or entry_row - row > TABLE_TRAVERSING_RANGE:
            raise DecodingFailedException('SOT not found')
        f, d = row_to_frame(row)
        row -= 1
        if f.id == can_id and _get_transfer_id(f) == transfer_id and d == direction:
            frames.insert(0, f)
            related_rows.insert(0, row)

    # Scanning forward looking for the last frame
    row = entry_row + 1
    while not _is_end_of_transfer(frames[-1]):
        f, d = row_to_frame(row)
        if f is None or row - entry_row > TABLE_TRAVERSING_RANGE:
            raise DecodingFailedException('EOT not found')
        row += 1
        if f.id == can_id and _get_transfer_id(f) == transfer_id and d == direction:
            frames.append(f)
            related_rows.append(row)

    # The transfer is now fully recovered
    for i, x in enumerate(frames):
        if not _frame_has_tail_byte(x):
            raise DecodingFailedException('Frame at index {} is missing tail byte'.format(i))

    tr = Transfer()
    tr.from_frames([Frame(x.id, x.data, canfd=x.canfd) for x in frames])

    full_text = get_payload_from_transfer(tr, frames)
    return related_rows, full_text
