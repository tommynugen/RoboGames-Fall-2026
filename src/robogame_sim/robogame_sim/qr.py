"""A small QR code maker, so the Teams tab can show its sign-up link as a
code to scan without another package to install.

    qr.matrix("http://192.168.1.20:8767/")  # rows of True (dark) / False
    qr.svg("http://192.168.1.20:8767/")     # the same, drawn as SVG

Byte mode, error correction level M (about 15% of the code can be lost),
versions 1 to 10: up to 213 bytes, plenty for a link. It follows the
QR code standard (ISO/IEC 18004) step by step: the bits, the Reed-Solomon
check bytes, the fixed patterns, the zig-zag placing and the best of the
eight masks.
"""

# For level M, versions 1 to 10: check bytes per block and blocks.
ECC_PER_BLOCK = (10, 16, 26, 18, 24, 16, 18, 22, 22, 26)
BLOCKS = (1, 1, 1, 2, 2, 4, 4, 4, 5, 5)
LEVEL_M = 0  # the two format bits for level M

MASKS = (
    lambda x, y: (x + y) % 2 == 0,
    lambda x, y: y % 2 == 0,
    lambda x, y: x % 3 == 0,
    lambda x, y: (x + y) % 3 == 0,
    lambda x, y: (x // 3 + y // 2) % 2 == 0,
    lambda x, y: x * y % 2 + x * y % 3 == 0,
    lambda x, y: (x * y % 2 + x * y % 3) % 2 == 0,
    lambda x, y: ((x + y) % 2 + x * y % 3) % 2 == 0,
)


def codewords(version):
    """Bytes the code holds at this version, data and check bytes."""
    bits = (16 * version + 128) * version + 64
    if version >= 2:
        count = version // 7 + 2
        bits -= (25 * count - 10) * count - 55
        if version >= 7:
            bits -= 36
    return bits // 8


def data_bytes(version):
    ecc = ECC_PER_BLOCK[version - 1] * BLOCKS[version - 1]
    return codewords(version) - ecc


def alignment(version):
    """Rows (and columns) of the small alignment squares' centres."""
    if version == 1:
        return []
    count = version // 7 + 2
    size = version * 4 + 17
    step = (version * 8 + count * 3 + 5) // (count * 4 - 4) * 2
    return [6] + sorted(size - 7 - i * step for i in range(count - 1))


# Reed-Solomon over GF(256), polynomial 0x11D.

def multiply(a, b):
    out = 0
    for i in range(7, -1, -1):
        out = (out << 1) ^ ((out >> 7) * 0x11D)
        out ^= ((b >> i) & 1) * a
    return out


def check_bytes(data, count):
    divisor = [0] * (count - 1) + [1]
    root = 1
    for _ in range(count):
        for j in range(count):
            divisor[j] = multiply(divisor[j], root)
            if j + 1 < count:
                divisor[j] ^= divisor[j + 1]
        root = multiply(root, 0x02)
    rest = [0] * count
    for byte in data:
        factor = byte ^ rest.pop(0)
        rest.append(0)
        for j, coefficient in enumerate(divisor):
            rest[j] ^= multiply(coefficient, factor)
    return rest


def encode(data, version):
    """The data bytes, padded, then split into blocks with their check
    bytes and woven together, as they go into the code."""
    length_bits = 8 if version <= 9 else 16
    bits = "0100" + format(len(data), f"0{length_bits}b")
    bits += "".join(format(byte, "08b") for byte in data)
    room = data_bytes(version) * 8
    bits += "0" * min(4, room - len(bits))
    bits += "0" * (-len(bits) % 8)
    padded = [int(bits[i:i + 8], 2) for i in range(0, len(bits), 8)]
    pad = 0
    while len(padded) < room // 8:
        padded.append((0xEC, 0x11)[pad % 2])
        pad += 1

    blocks_count = BLOCKS[version - 1]
    ecc = ECC_PER_BLOCK[version - 1]
    total = codewords(version)
    short_count = blocks_count - total % blocks_count
    short_length = total // blocks_count - ecc  # data bytes in a short block
    blocks, start = [], 0
    for i in range(blocks_count):
        length = short_length + (0 if i < short_count else 1)
        chunk = padded[start:start + length]
        start += length
        blocks.append((chunk, check_bytes(chunk, ecc)))
    out = []
    for i in range(short_length + 1):
        out += [chunk[i] for chunk, _ in blocks if i < len(chunk)]
    for i in range(ecc):
        out += [rest[i] for _, rest in blocks]
    return out


def format_bits(mask):
    data = LEVEL_M << 3 | mask
    rest = data
    for _ in range(10):
        rest = (rest << 1) ^ ((rest >> 9) * 0x537)
    return (data << 10 | rest) ^ 0x5412


def version_bits(version):
    rest = version
    for _ in range(12):
        rest = (rest << 1) ^ ((rest >> 11) * 0x1F25)
    return version << 12 | rest


class Grid:
    def __init__(self, version):
        self.size = version * 4 + 17
        self.dark = [[False] * self.size for _ in range(self.size)]
        self.fixed = [[False] * self.size for _ in range(self.size)]

    def put(self, x, y, dark):
        self.dark[y][x] = dark
        self.fixed[y][x] = True


def draw_fixed(grid, version):
    size = grid.size
    for i in range(size):
        grid.put(6, i, i % 2 == 0)
        grid.put(i, 6, i % 2 == 0)
    for cx, cy in ((3, 3), (size - 4, 3), (3, size - 4)):
        for dy in range(-4, 5):
            for dx in range(-4, 5):
                x, y = cx + dx, cy + dy
                if 0 <= x < size and 0 <= y < size:
                    grid.put(x, y, max(abs(dx), abs(dy)) not in (2, 4))
    places = alignment(version)
    last = len(places) - 1
    for i, cx in enumerate(places):
        for j, cy in enumerate(places):
            if (i, j) in ((0, 0), (0, last), (last, 0)):
                continue  # under a finder
            for dy in range(-2, 3):
                for dx in range(-2, 3):
                    grid.put(cx + dx, cy + dy, max(abs(dx), abs(dy)) != 1)
    draw_format(grid, 0)  # kept free now, written once the mask is known
    if version >= 7:
        bits = version_bits(version)
        for i in range(18):
            dark = (bits >> i) & 1 == 1
            a, b = size - 11 + i % 3, i // 3
            grid.put(a, b, dark)
            grid.put(b, a, dark)


def draw_format(grid, mask):
    bits = format_bits(mask)
    size = grid.size
    bit = [(bits >> i) & 1 == 1 for i in range(15)]
    for i in range(6):
        grid.put(8, i, bit[i])
    grid.put(8, 7, bit[6])
    grid.put(8, 8, bit[7])
    grid.put(7, 8, bit[8])
    for i in range(9, 15):
        grid.put(14 - i, 8, bit[i])
    for i in range(8):
        grid.put(size - 1 - i, 8, bit[i])
    for i in range(8, 15):
        grid.put(8, size - 15 + i, bit[i])
    grid.put(8, size - 8, True)


def draw_data(grid, data):
    size = grid.size
    bits = [(byte >> (7 - i)) & 1 == 1 for byte in data for i in range(8)]
    index = 0
    right = size - 1
    while right >= 1:
        if right == 6:
            right = 5  # past the timing column
        upward = (right + 1) & 2 == 0
        for step in range(size):
            y = size - 1 - step if upward else step
            for x in (right, right - 1):
                if not grid.fixed[y][x] and index < len(bits):
                    grid.dark[y][x] = bits[index]
                    index += 1
        right -= 2


def masked(grid, mask):
    rule = MASKS[mask]
    rows = [[grid.dark[y][x] != (not grid.fixed[y][x] and rule(x, y))
             for x in range(grid.size)] for y in range(grid.size)]
    out = Grid.__new__(Grid)
    out.size, out.dark, out.fixed = grid.size, rows, grid.fixed
    draw_format(out, mask)
    return out.dark


def penalty(rows):
    """The standard's score for how hard a code is to read: lower is
    better."""
    size = len(rows)
    score = 0
    lines = rows + [list(column) for column in zip(*rows)]
    finder = [True, False, True, True, True, False, True]
    for line in lines:
        run = 1
        for a, b in zip(line, line[1:]):
            if a == b:
                run += 1
                if run == 5:
                    score += 3
                elif run > 5:
                    score += 1
            else:
                run = 1
        padded = [False] * 4 + line + [False] * 4
        for i in range(len(padded) - 10):
            window = padded[i:i + 11]
            if (window[:7] == finder and not any(window[7:])) or \
                    (window[4:] == finder and not any(window[:4])):
                score += 40
    for y in range(size - 1):
        for x in range(size - 1):
            if rows[y][x] == rows[y][x + 1] == rows[y + 1][x] \
                    == rows[y + 1][x + 1]:
                score += 3
    dark = sum(map(sum, rows))
    total = size * size
    score += (abs(dark * 20 - total * 10) + total - 1) // total * 10 - 10
    return score


def matrix(text):
    """The QR code for text: rows of True for dark, without the quiet
    border."""
    data = text.encode("utf-8")
    for version in range(1, 11):
        header = 4 + (8 if version <= 9 else 16)
        if header + 8 * len(data) <= data_bytes(version) * 8:
            break
    else:
        raise ValueError(f"{len(data)} bytes is too long for a QR code here")
    grid = Grid(version)
    draw_fixed(grid, version)
    draw_data(grid, encode(data, version))
    return min((masked(grid, mask) for mask in range(8)), key=penalty)


def svg(text, module=8, border=4):
    """The QR code for text as an SVG picture, black on white, with the
    quiet border of `border` modules round it."""
    rows = matrix(text)
    side = len(rows) + 2 * border
    path = "".join(f"M{x + border},{y + border}h1v1h-1z"
                   for y, row in enumerate(rows)
                   for x, dark in enumerate(row) if dark)
    return (f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {side} '
            f'{side}" width="{side * module}" height="{side * module}" '
            f'shape-rendering="crispEdges"><rect width="{side}" '
            f'height="{side}" fill="#fff"/><path d="{path}" fill="#000"/>'
            "</svg>")
