import bluetooth
import subprocess

def main():

	target_address = None
	port = 32123
	data_size = 20
	passkey = "1234"
	
	target_address = '98:D3:31:F5:0C:5E'

	with open("output.csv","w"):
		pass

	sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)

	print("Conectando")
	sock.connect((target_address,port))
	print("Conectado")

	while True:
		data = sock.recv(data_size)
		if data:
			print(data.decode("utf-8"))

			if len(data) == 20:
				splitted = data.decode("utf-8").split(" ")
				print(splitted)

				with open("output.csv","a") as f:
					for word in splitted:
						f.write(word)
						if word != splitted(-1):
							f.write(',')
					f.write("\n")

			data = 0

if __name__ == '__main__':
	main()