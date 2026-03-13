import struct


class DataStructer:
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
                packet.extend(element.encode())
            elif isinstance(element, float):
                packet.extend(struct.pack('<f', element))
            elif isinstance(element, int):
                packet.append(element & 0xFF)
            elif isinstance(element, (bytes, bytearray)):
                packet.extend(element)
            else:
                raise ValueError(f"Unsupported data type: {type(element)}")
        return bytes(packet)
