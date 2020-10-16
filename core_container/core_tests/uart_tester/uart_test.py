import serial
import random
import sys
import time

NAMES = ['ross geller', 'rachel green', 'chandler bing', 'monica geller',
         'joey tribbiani', 'phoebe buffay']

def init_serial(port, baudrate):
  s = serial.Serial(port, baudrate, timeout=0.1)
  return s
# def init_serial

def get_test_name():
  n_len = len(NAMES)
  idx = random.randint(1, n_len) - 1
  string = NAMES[idx]
  str_len = len(NAMES[idx])
  return string, str_len
# def get_test_name

def serial_test(port, baudrate):
  ser = init_serial(port, baudrate)
  string, string_len = get_test_name()
  returns = []
  for i in range(string_len):
    ser.write(string[i].encode('utf8'))
    s = ser.read(1)
    returns.append(s.decode())
  returns = ''.join(returns)
  ser.close()
  print("Data sent: %s, Received: %s"%(string, returns))
  if (returns == string):
    return 0
  else:
    return 1
# def serial_test

if __name__ == "__main__":
  assert len(sys.argv) == 2, "name of port needed for the process"
  port = sys.argv[1]
  baudrate = 115200
  exit_code = serial_test(port, baudrate)
  assert exit_code == 0, ("Exited with exit code %d")%exit_code
  print("Test completed with exit code %d"%exit_code)
# main

