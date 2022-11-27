import serial

with serial.Serial('/dev/ttyS0', baudrate=115200) as ser:
    print(ser.name)

    r = ser.read_until(b'ready')
    print(r)
    ser.write(b'g')

    r = ser.read_until(b'transmitting')
    print(r)
    while True:
        r = ser.readline()
        if r.decode().startswith('sample'):
            print(r)
        if r.decode().startswith('timestamp'):
            print(r)
    r = ser.read_until(b'done')
    print(r)
