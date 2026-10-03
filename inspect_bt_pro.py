"""Read-only parser for this Java Object Serialization .pro capture."""
import struct
import sys
from pathlib import Path


class Parser:
    def __init__(self, data):
        assert data[:4] == b"\xac\xed\x00\x05"
        self.data = data
        self.pos = 4
        self.handles = {}
        self.next_handle = 0x7E0000

    def take(self, count):
        result = self.data[self.pos:self.pos + count]
        assert len(result) == count
        self.pos += count
        return result

    def num(self, fmt):
        return struct.unpack('>' + fmt, self.take(struct.calcsize(fmt)))[0]

    def utf(self):
        raw = self.take(self.num('H'))
        return raw.decode('utf-8', errors='replace')

    def handle(self, value):
        self.handles[self.next_handle] = value
        self.next_handle += 1
        return value

    def item(self):
        token = self.num('B')
        if token == 0x70:
            return None
        if token == 0x71:
            return self.handles[self.num('I')]
        if token == 0x74:
            return self.handle(self.utf())
        if token == 0x7C:
            length = self.num('Q')
            return self.handle(self.take(length).decode('utf-8', errors='replace'))
        if token == 0x72:
            name = self.utf()
            uid = self.num('Q')
            flags = self.num('B')
            fields = []
            descriptor = {'class': name, 'uid': uid, 'flags': flags, 'fields': fields, 'super': None}
            self.handle(descriptor)
            for _ in range(self.num('H')):
                kind = chr(self.num('B'))
                field_name = self.utf()
                type_name = self.item() if kind in ('L', '[') else None
                fields.append((kind, field_name, type_name))
            assert self.num('B') == 0x78
            descriptor['super'] = self.item()
            return descriptor
        if token == 0x73:
            descriptor = self.item()
            result = {'class': descriptor['class'], 'fields': {}, 'custom': []}
            self.handle(result)
            chain = []
            while descriptor is not None:
                chain.append(descriptor)
                descriptor = descriptor['super']
            for part in reversed(chain):
                for kind, field_name, _ in part['fields']:
                    result['fields'][field_name] = self.field(kind)
                if part['flags'] & 0x01:
                    while self.data[self.pos] != 0x78:
                        result['custom'].append(self.item())
                    self.pos += 1
            return result
        if token == 0x75:
            descriptor = self.item()
            length = self.num('I')
            result = {'array': descriptor['class'], 'items': []}
            self.handle(result)
            kind = descriptor['class'][1]
            for _ in range(length):
                result['items'].append(self.field(kind))
            return result
        if token == 0x77:
            return {'block': self.take(self.num('B')).hex()}
        if token == 0x7A:
            return {'block': self.take(self.num('I')).hex()}
        raise ValueError(f'Unsupported token {token:02x} at {self.pos - 1:04x}')

    def field(self, kind):
        if kind in ('L', '['):
            return self.item()
        return self.num({'B': 'b', 'C': 'H', 'D': 'd', 'F': 'f',
                         'I': 'i', 'J': 'q', 'S': 'h', 'Z': 'B'}[kind])


def show(value, depth=0):
    indent = '  ' * depth
    if isinstance(value, dict) and 'class' in value and 'fields' in value:
        print(indent + value['class'])
        for name, field in value['fields'].items():
            if isinstance(field, (dict, list)):
                print(indent + '  ' + name + ':')
                show(field, depth + 2)
            else:
                print(indent + f'  {name}: {field!r}')
        for custom in value.get('custom', []):
            print(indent + '  custom:')
            show(custom, depth + 2)
    elif isinstance(value, dict) and 'array' in value:
        print(indent + f"{value['array']} len={len(value['items'])}")
        for index, element in enumerate(value['items']):
            print(indent + f'  [{index}]:')
            show(element, depth + 2)
    else:
        print(indent + repr(value))


if __name__ == '__main__':
    parser = Parser(Path(sys.argv[1]).read_bytes())
    project = parser.item()
    print(f'parsed={parser.pos} total={len(parser.data)}')
    show(project)
