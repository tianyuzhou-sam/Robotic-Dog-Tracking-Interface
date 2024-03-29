#!/usr/bin/python3
import socket
import time
import json
import numpy as np

"""
    This is an example where you can launch a subscriber via TCP/IP.
    You can receive data from the publisher if there exists a connection between this subscriber and the publisher.
"""

def subscriber_tcp_main(config_file_name):

    # Read the configuration from the json file
    json_file = open(config_file_name)
    config_data = json.load(json_file)

    # IP for publisher
    HOST = config_data['QUALISYS']['IP_STATES_ESTIMATION']
    # Port for publisher (non-privileged ports are > 1023)
    # Remember to convert the string to an integer
    PORT = int(config_data['QUALISYS']['PORT_STATES_ESTIMATION'])

    # for a single message, how many integers
    data_number_integer = int(config_data['QUALISYS']['DATA_NUMBERS_STATES_ESTIMATION'])

    # Create a TCP/IP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # Connect the socket to the port where the server is listening
    server_address = (HOST, PORT)
    sock.connect(server_address)
    print("Connected to", server_address)

    # buffer size
    buffersize = int(config_data['QUALISYS']['DATA_BYTES_MAX'])
    
    while True:
        time.sleep(0.2)
        subscriber_callback(sock, buffersize, data_number_integer)


def subscriber_callback(sock, buffersize, data_number_integer):
    """
    Receives the data from the publisher.
    Args:
        sock:
            The TCP/IP socket class.
    Returns:
        return_flag:
            The return_flag is a boolean variable which shows if 
            the subscriber receives the data. True for yes; False for no.
    Returns the boolean flag (True). You can also do something else
    when the subscriber doesn't receive the data.
    """

    msg = sock.recv(buffersize)
    if msg:
        data = np.frombuffer(msg, dtype=float)
        num_data_group = int(np.size(data)/data_number_integer)
        data_all = data[-data_number_integer*num_data_group:]
        print("Received the data from the publisher.")
        # print(num_data_group)
        # print(data_all)

        data_for_LLC = data[-data_number_integer:]
        # 2-D numpy array, 3 by 1, [px; py; pz], in meters
        posi_now = np.reshape(data_for_LLC[0:3], (-1, 1))
        print(posi_now)
        # 1-D numpy array to list, [w, x, y, z]
        Rot_Mat = data_for_LLC[3:data_number_integer-1].reshape(3, 3)
        print(Rot_Mat)

        return_flag = True
    else:
        print("Didn't receive the data from the publisher.")
        return_flag = False
        # do something else
    return return_flag


if __name__ == "__main__":
    config_file_name = 'config_aimslab.json'
    subscriber_tcp_main(config_file_name)
