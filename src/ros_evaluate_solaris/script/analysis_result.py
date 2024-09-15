# analysis the data in record folder, and output result to result folder
import numpy as np
import pandas as pd
import os

def read_file(file):
    fr = open(file, 'r')
    datas = fr.readlines()
    # print(len(datas))
    # first calc the num of chain
    st = set()
    for data in datas:
        st.add(data.split('-')[0].split(':')[1])
    # print(len(st))
    return len(st), datas

def get_info(line):
    chain_id = int(line.split('-')[0].split(':')[1])
    node_id = int(line.split('-')[1].split(':')[1])
    count = int(line.split('-')[2].split(':')[1])
    chain_type = line.split('-')[3].split(':')[0]
    time = int(line.split('-')[3].split(':')[1])
    return chain_id, node_id, count, chain_type, time

def to_label(latency: np.ndarray) -> str:
    # latency = latency[[not pd.isnull(_) for _ in latency]]

    # check count
    # print(len(latency))
    # transfer to ms
    # print(latency * 1.0e-6)

    # all we need flag
    latency_percentile = np.percentile(latency * 1.0e-6, 0.99)
    latency_std = np.std(latency * 1.0e-6)
    latency_min = np.min(latency * 1.0e-6)
    latency_avg = np.average(latency * 1.0e-6)
    latency_max = np.max(latency * 1.0e-6)
    
    label = (
        'min: {:.2f} ms\n'.format(latency_min)
        + 'avg: {:.2f} ms\n'.format(latency_avg)
        + 'max: {:.2f} ms\n'.format(latency_max)
        + 'p99: {:.2f} ms\n'.format(latency_percentile)
        + 'std: {:.2f} \n'.format(latency_std)
    )
    return label

def calc_latency(num_of_chain, datas):
    latency = []
    temp = []
    for i in range(num_of_chain):
        latency.append([])
        temp.append({})
    # print(latency)
    
    # for ith in range(num_of_chain):
    
    for jth in datas:
        chain_id, node_id, count, chain_type, time = get_info(jth)
        chain_id -= 1
        if chain_type == 'start':
            temp[chain_id][count] = time
            # temp[chain_id] = time
        else:
            # latency[chain_id].append((time-temp[chain_id]))
            ltc = time - temp[chain_id][count]
            latency[chain_id].append(ltc)
    
    # print(latency)
    for i in range(num_of_chain):
        print(to_label(np.array(latency[i])))
    
    

def main():
    os.system("cd ../../..")
    # os.system("pwd")
    # read_file('record/RECORD')
    num_of_chain, datas = read_file('--ros-args')
    calc_latency(num_of_chain, datas)

if __name__ == '__main__':
    main()