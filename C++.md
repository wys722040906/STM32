# C++

# 延时

```
#include <iostream>
#include <chrono>
#include <thread>

int main() {
    std::cout << "开始延时" << std::endl;

    // 延时5秒
    std::this_thread::sleep_for(std::chrono::seconds(5));

    std::cout << "延时结束" << std::endl;
    return 0;
}
```

## chrono--时间

- 持续时间、时钟和时间点,测量代码的执行时间、创建定时器

### std::chrono::duration

### std::chrono::time_point

## thread--多线程编程

- 创建和管理线程的类和函数

### std::thread

### std::mutex

### std::condition_variable

# 进制表示

## 流格式改变

- std::hex
- std::oct
- std::oct

```
#include <iostream>
#include <string>

// 自定义函数，将整数转换为任意进制字符串
std::string to_base(int value, int base) {
    const char* digits = "0123456789ABCDEF";
    std::string result;
    while (value > 0) {
        result.insert(result.begin(), digits[value % base]);
        value /= base;
    }
    return result.empty() ? "0" : result;
}

int main() {
    int num = 255;
    int base = 16; // 设置目标进制
    std::cout << "十进制数 " << num << " 的 " << base << " 进制表示为： " << to_base(num, base) << std::endl;
    return 0;
}
```

