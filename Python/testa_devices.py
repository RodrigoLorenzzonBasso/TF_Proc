import bluetooth

def main():
	print("Hello World!")

	target_name = "HC-06"
	target_address = None
	port = 32123
	data_size = 1024

	nearby_devices = bluetooth.discover_devices(lookup_names=True)
	print("found %d devices" % len(nearby_devices))

	for bdaddr in nearby_devices:
		if target_name == bluetooth.lookup_name(bdaddr):
			target_address = bdaddr
			break

	if target_address is not None:
		sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
		sock.connect((target_address,port))

		while True:
			data = sock.recv(data_size)
			if data:
				print(data.decode("utf-8"))
	else:
		print("No device found")

if __name__ == '__main__':
	main()