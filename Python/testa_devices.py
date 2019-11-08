import bluetooth
import subprocess

def main():
	print("Hello World!")

	target_name = "HC-06"
	target_address = None
	port = 32123
	data_size = 20
	passkey = "1234"

	subprocess.call("kill -9 `pidof bluetooth-agent`",shell=True)

	status = subprocess.call("bluetooth-agent " + passkey + " &",shell=True)

	nearby_devices = bluetooth.discover_devices(lookup_names=True)
	print("found %d devices" % len(nearby_devices))

	print(nearby_devices)

	for device in nearby_devices:
		if target_name == bluetooth.lookup_name(device[0]):
			target_address = device[0]
			break
	
	target_address = '98:D3:31:F5:0C:5E'

	if target_address is not None:
		sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
		sock.connect((target_address,port))

		while True:
			data = sock.recv(data_size)
			if data:
				print(data.decode("utf-8"))
				data = 0
	else:
		print("No device found")

if __name__ == '__main__':
	main()