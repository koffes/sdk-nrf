import serial
import time

def send_shell_command(command, port, sleep_time=3, baudrate=9600, timeout=1):
    """
    Sends a shell command to an embedded device via UART.

    Args:
        command (str): The shell command to send.
        port (str): The serial port (e.g., '/dev/ttyUSB0', 'COM3').
        baudrate (int): The baud rate for the serial communication (default: 9600).
        timeout (int): The timeout for the serial communication in seconds (default: 1).

    Returns:
        str: The response from the device.
    """
    try:
        # Open the serial port
        with serial.Serial(port, baudrate, timeout=timeout) as ser:
            # Send the command to the device
            ser.write((command + '\n').encode())

            # Give the device some time to respond
            time.sleep(sleep_time)

            # Read the response from the device
            response = ser.read_all().decode()

            return response

    except serial.SerialException as e:
        print(f"Error: {e}")
        return None

# Example usage:
# response = send_shell_command('ls', '/dev/ttyUSB0')
# print(response)

response = send_shell_command('bct adv_name anvi-audio-is-best-audio', '/dev/ttyACM0')
response = send_shell_command('bct preset 48_4_1 0', '/dev/ttyACM0')
#response = send_shell_command('bct num_bises 1 0 0', '/dev/ttyACM0')

response = send_shell_command('bct file select right-channel/generated_lc3/48000hz/right-channel_48kHz_left_96kbps.lc3 0 0 0 1', '/dev/ttyACM0')
response = send_shell_command('bct file select left-channel/generated_lc3/48000hz/left-channel_48kHz_left_96kbps.lc3 0 0 1 1', '/dev/ttyACM0')

response = send_shell_command('bct start', '/dev/ttyACM0')
