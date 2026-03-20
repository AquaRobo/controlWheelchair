import struct

class DataStructer:
    _FLOAT_SIZE = struct.calcsize('<f')

    @staticmethod
    def to_bytes(data: list | bytes | bytearray) -> bytes:
        return DataStructer.__structData(data)

    @staticmethod
    def to_byte_list(data: list | bytes | bytearray) -> list[int]:
        return list(DataStructer.__structData(data))

    @staticmethod
    def __structData(data: list | bytes | bytearray) -> bytes:
        if isinstance(data, (bytes, bytearray)):
            return bytes(data)
        if not isinstance(data, list):
            raise ValueError("The payload should be a list")

        packet = bytearray()
        for element in data:
            if isinstance(element, str):
                encoded = element.encode()
                if len(encoded) > DataStructer._FLOAT_SIZE:
                    raise ValueError("String elements should have at most 4 bytes")
                packet.extend(encoded.ljust(DataStructer._FLOAT_SIZE, b'\x00'))
            elif isinstance(element, float):
                packet.extend(struct.pack('<f', element))
            elif isinstance(element, int):
                packet.extend(struct.pack('<i', element))
            elif isinstance(element, (bytes, bytearray)):
                raw_element = bytes(element)
                if len(raw_element) > DataStructer._FLOAT_SIZE:
                    raise ValueError("Byte elements should have at most 4 bytes")
                packet.extend(raw_element.ljust(DataStructer._FLOAT_SIZE, b'\x00'))
            else:
                raise ValueError(f"Unsupported data type: {type(element)}")
        return bytes(packet)
