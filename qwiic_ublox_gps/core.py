"""The core structure definitions"""

import struct
from collections import namedtuple
from typing import List, Iterator, Union

__all__ = ['PadByte', 'Field', 'Flag', 'BitField', 'RepeatedBlock', 'Message', 'Cls', 'Parser', ]


class PadByte:
    """A padding byte, used for padding the messages.

    The number of repeats needs to be used carefully...
    If you want 6 total pad bytes, the pad byte is repeated 5 times.

    If this proves confusing it may be changed in the future.
    """
    __slots__ = ['repeat', ]

    def __init__(self, repeat: int = 0):
        self.repeat = repeat

    @property
    def repeated_block(self):
        return False

    @property
    def fmt(self):
        """Return the format char for use with the struct package"""
        return 'x' * (self.repeat + 1)

    @staticmethod
    def parse(_it: Iterator):
        """Discard the padding bytes"""
        return None, None


class Field:
    """A field type that is used to describe most `normal` fields.

    The descriptor code is as per the uBlox data sheet available;
    https://www.u-blox.com/sites/default/files/products/documents/u-blox8-M8_ReceiverDescrProtSpec_%28UBX-13003221%29_Public.pdf

    Field types that are variable length are not supported at this stage.

    In future that support may be added but it would probably use a different field constructor...
    """
    __types__ = {'U1': 'B', 'I1': 'b',
                 'U2': 'H', 'I2': 'h',
                 'U4': 'I', 'I4': 'i', 'R4': 'f',
                 'R8': 'd', 'C': 'c'}
    __slots__ = ['name', '_type', ]

    def __init__(self, name: str, type_: str):
        self.name = name

        if type_ not in Field.__types__:
            raise ValueError('The provided _type of {} is not valid'.format(type_))
        self._type = type_

    @property
    def repeated_block(self):
        return False

    @property
    def fmt(self):
        """Return the format char for use with the struct package"""
        return Field.__types__[self._type]

    def parse(self, it: Iterator) -> tuple:
        """Return a tuple representing the provided value/s"""
        resp = []
        value = next(it)

        if self._type in ['U1', 'I1', 'U2', 'I2', 'U4', 'I4', ]:
            resp = int(value)

        if self._type in ['R4', 'R8', ]:
            resp = float(value)

        if self._type == 'C':
            resp = value

        return self.name, resp


class Flag:
    """A flag within a bit field.

    The start and stop indexes are used in a similar way to list indexing.
    They are zero indexed and the stop is exclusive.

    So for flag at bit zero that is one bit wide the constructor would be;
    Flag('your flag name', 0, 1)

    This class does a basic check against the implied field width eg. < 4*8, but any
    strict checking is done within classes that use this. For example you can set a
    start and stop > 8 even if the bit field is only 8 bits wide.
    """
    __slots__ = ['name', '_start', '_stop', '_mask', ]

    def __init__(self, name: str, start: int, stop: int):
        self.name = name

        if 0 > start:
            raise ValueError('The start index must be greater than 0 not {}'.format(start))

        if start > stop:
            raise ValueError('The start index, {}, must be higher than the stop index, {}'.format(start, stop))

        if stop > 4 * 8:
            raise ValueError('The stop index must be less than 4 bytes wide not {}'.format(stop))

        self._start = start
        self._stop = stop

        self._mask = 0x00
        for i in range(start, stop):
            self._mask |= 0x01 << i

    def parse(self, value) -> tuple:
        """Return a tuple representing the provided value"""
        return self.name, (value & self._mask) >> self._start


class BitField:
    """A bit field type made up of flags.

    The bit field uses the types described within the uBlox data sheet:
    https://www.u-blox.com/sites/default/files/products/documents/u-blox8-M8_ReceiverDescrProtSpec_%28UBX-13003221%29_Public.pdf

    Flags should be passed within the constructor and should not be added after
    the class has been created. The constructor does check whether the flag field
    indexes would imply that the BitField is wider than specified. If this is found
    it will raise a ValueError.

    """

    __slots__ = ['name', '_type', '_subfields', '_nt', ]
    __types__ = {'X1': 'B', 'X2': 'H', 'X4': 'I'}

    # noinspection PyProtectedMember
    def __init__(self, name: str, type_: str, subfields: List[Flag]):
        self.name = name

        if type_ not in BitField.__types__:
            raise ValueError('The provided _type of {} is not valid'.format(type_))
        self._type = type_

        self._subfields = subfields

        if type_ == 'X1':
            width = 1
        elif type_ == 'X2':
            width = 2
        else:
            width = 4

        for sf in subfields:
            if sf._stop > (width * 8):
                raise ValueError('{} stop index of {} is wider than the implicit width of {} bytes'.format(
                    sf.__class__.__name__, sf._stop, width
                ))

        self._nt = namedtuple(self.name, [f.name for f in self._subfields])

    @property
    def repeated_block(self):
        return False

    @property
    def fmt(self) -> str:
        """Return the format char for use with the struct package"""
        return BitField.__types__[self._type]

    def parse(self, it: Iterator) -> namedtuple:
        """Return a named tuple representing the provided value"""
        value = next(it)
        return self.name, self._nt(**{k: v for k, v in [x.parse(value) for x in self._subfields]})


class RepeatedBlock:
    """Defines a repeated block of Fields within a UBX Message

    """
    __slots__ = ['name', '_fields', 'repeat', '_nt', ]

    def __init__(self, name: str, fields: List[Union[Field, BitField, PadByte]]):
        self.name = name
        self._fields = fields
        self.repeat = 0
        self._nt = namedtuple(self.name, [f.name for f in self._fields if hasattr(f, 'name')])

    @property
    def repeated_block(self):
        return True

    @property
    def fmt(self):
        """Return the format string for use with the struct package."""
        return ''.join([field.fmt for field in self._fields]) * (self.repeat + 1)

    def parse(self, it: Iterator) -> tuple:
        """Return a tuple representing the provided value/s"""
        resp = []
        for i in range(self.repeat + 1):
            resp.append(self._nt(**{k: v for k, v in [f.parse(it) for f in self._fields] if k is not None}))

        return self.name, resp


class Message:
    """Defines a UBX message.

    The Messages are described in the data sheet:
    https://www.u-blox.com/sites/default/files/products/documents/u-blox8-M8_ReceiverDescrProtSpec_%28UBX-13003221%29_Public.pdf

    The supplied name should be upper case. eg. PVT

    The fields that make up message should be passed into the constructor as a list with the fields
    in the correct order. Modifying the list or the fields after construction is not supported as it
    has side effects.

    The id is only allowed to be one byte wide so 0x00 <= id_ <= 0xFF values outside this range
    will raise a ValueError

    """
    __slots__ = ['_id', 'name', '_fields', '_nt', '_repeated_block', ]

    def __init__(self, id_: int, name: str, fields: list):
        if id_ < 0:
            raise ValueError('The _id must be >= 0, not {}'.format(id_))

        if id_ > 0xFF:
            raise ValueError('The _id must be <= 0xFF, not {}'.format(id_))

        self._id = id_
        self.name = name
        self._fields = fields
        self._nt = namedtuple(self.name, [f.name for f in self._fields if hasattr(f, 'name')])
        self._repeated_block = None

        for field in fields:
            if field.repeated_block:
                if self._repeated_block is not None:
                    raise ValueError('Cannot assign multiple repeated blocks to a message.')
                self._repeated_block = field

    @property
    def id_(self):
        """Public read only access to the message id"""
        return self._id

    @property
    def fmt(self):
        """Return the format string for use with the struct package."""
        return ''.join([field.fmt for field in self._fields])

    def parse(self, payload: bytes) -> namedtuple:
        """Return a named tuple parsed from the provided payload.

        If the provided payload is not the same length as what is implied by the format string
        then a ValueError is raised.
        """

        payload_len = len(payload)

        try:
            self._repeated_block.repeat = 0
        except AttributeError:
            pass

        while True:
            fmt_len = struct.calcsize(self.fmt)

            if fmt_len == payload_len:
                break

            if fmt_len > payload_len:
                raise ValueError('The payload length does not match the length implied by the message fields. ' +
                                 'Expected {} actual {}'.format(struct.calcsize(self.fmt), len(payload)))

            try:
                self._repeated_block.repeat += 1
            except AttributeError:
                raise ValueError('The payload length does not match the length implied by the message fields. ' +
                                 'Expected {} actual {}'.format(struct.calcsize(self.fmt), len(payload)))

        it = iter(struct.unpack(self.fmt, payload))

        return self.name, self._nt(**{k: v for k, v in [f.parse(it) for f in self._fields] if k is not None})


class Cls:
    """Defines a UBX message class.

    The Classes are described in the data sheet:
    https://www.u-blox.com/sites/default/files/products/documents/u-blox8-M8_ReceiverDescrProtSpec_%28UBX-13003221%29_Public.pdf

    The messages within the class can be provided via the constructor or via the `register_msg` method.

    The id_ should not be modified after registering the class with the parser.
    """
    __slots__ = ['_id', 'name', '_messages', ]

    def __init__(self, id_: int, name: str, messages: List[Message]):
        if id_ < 0:
            raise ValueError('The _id must be >= 0, not {}'.format(id_))

        if id_ > 0xFF:
            raise ValueError('The _id must be <= 0xFF, not {}'.format(id_))

        self._id = id_

        self.name = name

        self._messages = {}
        for msg in messages:
            self._messages[msg.id_] = msg

    @property
    def id_(self):
        """Public read only access to the class id"""
        return self._id

    def __contains__(self, item):
        return self._messages.__contains__(item)

    def __getitem__(self, item) -> Message:
        try:
            return self._messages[item]
        except KeyError:
            raise KeyError("A message of id {} has not been registered within {!r}".format(
                item, self
            ))

    def register_msg(self, msg: Message):
        """Register a message type."""
        # noinspection PyProtectedMember
        self._messages[msg._id] = msg

    def parse(self, msg_id, payload):
        """Return a named tuple parsed from the provided payload.

        If the provided payload is not the same length as what is implied by the format string
        then a ValueError is raised.

        """
        name, nt = self._messages[msg_id].parse(payload)
        return self.name, name, nt


class Parser:
    """A lightweight UBX message parser.

    This class is designed to contain a set of message classes, when a stream is passed via the `receive_from`
    method the stream is read until the PREFIX is found, the message type is matched to the registered class
    and message id's, the length read, checksum checked and finally the message is deconstructed and returned
    as a named tuple.

    This is powered by the inbuilt struct package for the heavy lifting of the message decoding and there are
    no external dependencies.

    Message classes can be passed via the constructor or the `register_cls` method.

    The `receive_from` method doesn't care about the underlying type of the provided stream, so long as it
    supports a `read` method and returns bytes. It is up to the implementation to determine how to get the
    UBX packets from the device. The typical way to do this would be a serial package like pyserial, see the
    examples file.
    """
    PREFIX = bytes((0xB5, 0x62))

    def __init__(self, classes: List[Cls]):
        self._input_buffer = b''

        self.classes = {}
        for cls in classes:
            self.classes[cls._id] = cls

    def register_cls(self, cls: Cls):
        """Register a message  class."""
        self.classes[cls.id_] = cls

    def receive_from(self, stream) -> namedtuple:
        """Receive a message from a stream and return as a namedtuple.
        raise IOError or ValueError on errors.
        """
        while True:
            # Search for the prefix
            buff = self._read_until(stream, terminator=self.PREFIX)
            if buff[-2:] == self.PREFIX:
                break

        # read the first four bytes
        buff = stream.read(4)

        if len(buff) != 4:
            raise IOError("A stream read returned {} bytes, expected 4 bytes".format(len(buff)))

        # convert them into the packet descriptors
        msg_cls, msg_id, length = struct.unpack('BBH', buff)

        # check the packet validity
        if msg_cls not in self.classes:
            raise ValueError("Received unsupported message class of {:x}".format(msg_cls))

        if msg_id not in self.classes[msg_cls]:
            raise ValueError("Received unsupported message id of {:x} in class {:x}".format(
                msg_id, msg_cls))

        # Read the payload
        buff += stream.read(length)
        if len(buff) != (4 + length):
            raise IOError("A stream read returned {} bytes, expected {} bytes".format(
                len(buff), 4 + length))

        # Read the checksum
        checksum_sup = stream.read(2)
        if len(checksum_sup) != 2:
            raise IOError("A stream read returned {} bytes, expected 2 bytes".format(len(buff)))

        checksum_cal = self._generate_fletcher_checksum(buff)
        if checksum_cal != checksum_sup:
            raise ValueError("Checksum mismatch. Calculated {:x} {:x}, received {:x} {:x}".format(
                checksum_cal[0], checksum_cal[1], checksum_sup[0], checksum_sup[1]
            ))

        return self.classes[msg_cls].parse(msg_id, buff[4:])

    @staticmethod
    def _read_until(stream, terminator: bytes, size=None):
        """Read from the stream until the terminator byte/s are read.
        Return the bytes read including the termination bytes.
        """
        term_len = len(terminator)
        line = bytearray()
        while True:
            c = stream.read(1)
            if c:
                line += c
                if line[-term_len:] == terminator:
                    break
                if size is not None and len(line) >= size:
                    break
            else:
                break

        return bytes(line)

    @staticmethod
    def _generate_fletcher_checksum(payload: bytes) -> bytes:
        """Return the checksum for the provided payload"""
        check_a = 0
        check_b = 0

        for char in payload:
            check_a += char
            check_a &= 0xFF

            check_b += check_a
            check_b &= 0xFF

        return bytes((check_a, check_b))
