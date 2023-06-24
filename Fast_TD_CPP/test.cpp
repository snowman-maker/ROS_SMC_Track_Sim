#include "FastTD.h"
#include "iostream"
#include "vector"
#include "cmath"
#include "iostream"
#include "fstream"
#include "string"

void WriteToFile(std::string file_to_write, 
                 std::vector<double> v, 
                 std::vector<double> x1,
                 std::vector<double> x2)
{
    std::ofstream outfile; // 创建一个输出文件流对象

    // 打开文件，如果文件不存在则创建新文件
    outfile.open(file_to_write);

    if (outfile.is_open()) { // 检查文件是否成功打开
        for(int k=0; k<v.size(); k++){
            outfile << x1[k] << ", " << x2[k] << ", " << v[k] << std::endl;
        } // 将内容写入文件
        outfile.close(); // 关闭文件流
        std::cout << "写入文件成功！" << std::endl;
    } else {
        std::cout << "无法打开文件！" << std::endl;
    }
}

int main()
{
    double h = 0.2;
    double r =1.0;

    int n = 200;

    FAST_TD td(r,h);

    std::vector<double> v;
    std::vector<double> x1;
    std::vector<double> x2;

    for(int k = 0; k < n; k++){
        v.push_back(0);
        x1.push_back(0);
        x2.push_back(0);
    }
    v[0] = 1;

    
    for(int k = 1; k < n; k++){
        v[k] = (sin(k * h) + 0.01 + 1);
        x1[0] = v[1];
        x2[0] = 0;
        td.track(v[k-1], x1[k-1]);
        x1[k] = (td.get_x1());
        x2[k] = (td.get_x2());
        std::cout << "origin: " << v[k-1] << ", track: " << x1[k] << ", " << x2[k] << std::endl;
    }
    std::string file_to_write = "../result.txt";
    WriteToFile(file_to_write, v, x1, x2);

    return 0;
}