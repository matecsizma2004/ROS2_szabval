"""ROS-független segédfüggvények: rács <-> kép, szakaszkeresés, koordináták."""

BLACK = 0
WHITE = 255


def parse_grid(rows):
    """'0'/'1' karakterláncok listájából 2D bool rácsot készít (True = fekete)."""
    if not rows:
        raise ValueError('A rács üres.')
    width = len(rows[0])
    grid = []
    for i, row in enumerate(rows):
        if len(row) != width or set(row) - {'0', '1'}:
            raise ValueError(f'Hibás rácssor ({i}.): "{row}"')
        grid.append([c == '1' for c in row])
    return grid


def grid_to_pixels(grid):
    """Rácsból mono8 pixeltömb (sorfolytonos): fekete = 0, fehér = 255."""
    return bytes(BLACK if cell else WHITE for row in grid for cell in row)


def image_to_grid(data, width, height):
    """mono8 képadatból bool rács; 128 alatti pixel = fekete."""
    if len(data) < width * height:
        raise ValueError('A képadat rövidebb, mint width * height.')
    return [[data[r * width + c] < 128 for c in range(width)] for r in range(height)]


def find_strokes(grid):
    """Soronként (fentről lefelé), balról jobbra a fekete szakaszok.

    Visszaad: [(sor, kezdő_oszlop, záró_oszlop), ...]. Az 1 hosszú
    (magányos) cellákat kihagyja, mert ilyenek a feladat szerint nincsenek.
    """
    strokes = []
    for r, row in enumerate(grid):
        c = 0
        while c < len(row):
            if row[c]:
                start = c
                while c + 1 < len(row) and row[c + 1]:
                    c += 1
                if c > start:
                    strokes.append((r, start, c))
            c += 1
    return strokes


def cell_center(row, col, sheet_x, sheet_y, z, cell_size):
    """A (sor, oszlop) cella középpontja world koordinátákban."""
    return (sheet_x - (row + 0.5) * cell_size,
            sheet_y - (col + 0.5) * cell_size,
            z)
