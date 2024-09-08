# analysis the data in record folder, and output result to result folder
import numpy as np
import pandas as pd
import os

def read_file(file):
    fr = open(file, 'r')
    datas = fr.readlines()
    print(len(datas))
    # first calc the num of chain
    st = set()
    for data in datas:
        st.add(data.split('-')[0].split(':')[1])
    print(len(st))
    pass

def main():
    os.system("cd ../../..")
    # os.system("pwd")
    read_file('record/RECORD')

if __name__ == '__main__':
    main()