import math

# THIS SCRIPT IS AI GENERATED FROM MY ALREADY EXISTING C CODE

# --- Configuration (matches your C code) ---
SINE_TABLE_SIZE = 256
SWOOP_TABLE_SIZE = 6500
CHIRP_TABLE_SIZE = 6500

# DDS parameters
TWO_32 = 4294967296.0  # 2^32
FS = 50000.0  # Sampling frequency

# --- Table Generation Functions ---

def generate_sin_table():
    """ Generates a NORMALIZED sine wave table for DDS. """
    table = []
    # The size of the table, 256 steps
    SINE_TABLE_SIZE = 256
    for i in range(SINE_TABLE_SIZE):
        # 1. Get a pure sine value from -1.0 to 1.0
        float_val = math.sin(i * 2.0 * math.pi / SINE_TABLE_SIZE)
        # 2. Scale it to the full range of a short int (-32767 to 32767)
        fixed_point_val = int(float_val * 32767.0)
        table.append(fixed_point_val)
    return table

def generate_swoop_table():
    """ Generates the frequency swoop table (phase increments). """
    table = []
    for kk in range(SWOOP_TABLE_SIZE):
        # Logic from your C code
        freq = -260.0 * math.sin(-0.000483 * kk) + 1740.0
        phase_increment = int((freq * TWO_32) / FS)
        table.append(phase_increment)
    return table

def generate_chirp_table():
    """ Generates the frequency chirp table (phase increments). """
    table = []
    for gg in range(CHIRP_TABLE_SIZE):
        # Logic from your C code
        freq = ((gg * gg) / 8450.0) + 2000.0
        phase_increment = int((freq * TWO_32) / FS)
        table.append(phase_increment)
    return table

# --- File Writing Function ---

def write_c_header(filename, array_name, data_type, data, items_per_line=12):
    """ Writes a list of integers to a C header file. """
    with open(filename, 'w') as f:
        # Write include guard
        guard = filename.upper().replace('.', '_')
        f.write(f'#ifndef {guard}\n')
        f.write(f'#define {guard}\n\n')

        # Write array declaration
        f.write(f'// Generated tablecalc.py\n')
        f.write(f'static const {data_type} {array_name}[{len(data)}] = {{\n    ')

        # Write data
        for i, val in enumerate(data):
            f.write(f'{val}, ')
            if (i + 1) % items_per_line == 0:
                f.write('\n    ')

        # Close array and include guard
        f.write('\n};\n\n')
        f.write(f'#endif // {guard}\n')
    print(f'Successfully generated {filename}')

# --- Main Execution ---

if __name__ == '__main__':
    print("Generating lookup tables...")

    # Generate data
    sin_data = generate_sin_table()
    swoop_data = generate_swoop_table()
    chirp_data = generate_chirp_table()

    # Write to header files
    # We use stdint types for clarity (fix15 is a 16-bit signed int)
    write_c_header('sin_table.h', 'sin_table', 'int16_t', sin_data)
    # Phase increments are large unsigned integers
    write_c_header('swoop_table.h', 'swoop_table', 'uint32_t', swoop_data)
    write_c_header('chirp_table.h', 'chirp_table', 'uint32_t', chirp_data)

    print("\nDone!")
