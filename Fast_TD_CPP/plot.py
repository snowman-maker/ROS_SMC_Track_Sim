import matplotlib.pyplot as plt

if __name__ == '__main__':
    file_to_read = "./result.txt"
    data = []
    with open(file_to_read, 'r') as file:
        for line in file:
            row = line.strip().split(",")  # 去除换行符并按逗号分隔行数据
            data.append([float(i) for i in row])
    file.close()
    for i in range(len(data)):
        v = data[i][0]
        x1 = data[i][1]
        x2 = data[i][2]        
    fig = plt.figure()
    plt.plot(v, color='red', label='origin')
    plt.plot(x1, color='green', label='track')
    plt.plot(x2, color='blue', label='track_dot')

    plt.legend()
    plt.show()
    
