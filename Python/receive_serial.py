import serial

data_size = 16
port = 'COM13'
baud_rate = 38400

with open ("output.csv","w"):
    print("Novo arquivo criado")

with serial.Serial(port,baud_rate,timeout=None) as s:

    n = 1

    while True:
        byte_data = s.read(data_size)
        splitted = byte_data.decode("utf-8").split(" ")
        print(splitted)

        with open("output.csv","a") as f:
            f.write(str(n) + ',')
            n += 1
            for word in splitted:
                f.write(word)
                if word != splitted[-1]:
                    f.write(',')
            f.write("\n")

print("no device found")

