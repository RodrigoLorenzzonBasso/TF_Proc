
data = "24.5 50.0 1234.5"

splitted = data.split(" ")

print(data)
print(splitted)

with open ("output.csv","w"):
    pass

for i in range(5):
    with open("output.csv","a") as f:
        for word in splitted:
            f.write(word)
            if word != splitted[-1]:
                f.write(',')
        f.write("\n")


b_data = b'asdsasdas\n'

print(b_data)
print(len(b_data))
decode = b_data.decode("utf-8")
print(decode)
print(len(decode))
print(type(decode))
