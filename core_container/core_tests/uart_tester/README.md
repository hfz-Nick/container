# UART Tester

## 1. Brief

The python code is used to test the UART ports in UUSV.

## 2. Dependencies

- python3

- pyserial (`python3 -m pip install pyserial`)

## 3. How to Use

1. Prepare an STM32F4 Discovery board, flash the code that will return whatever UART message sent to it, and hook up the UART pins to the UART ports that we are going to test. [STM Code](http://192.168.69.40/uusv/STM32/uusv_usart_test)

2. Run the script with the port you want to test as an argument, for instance:

  ```
  python3 uart_test.py /dev/nvt_serial_md
  ```

3. If test is successful, it will be exited with code 0, otherwise read the code and figure out the error.
