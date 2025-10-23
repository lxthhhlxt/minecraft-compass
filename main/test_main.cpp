#include <stdio.h>
#include <string.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "driver/uart.h"
#include "hal/i2c_types.h"
#include "mpu6050.hpp"
#include "qmc5883p.hpp"
#include "soc/gpio_num.h"
#include "esp_timer.h"
#include "led_strip.h"
#include "compressed_led.hpp"
#include "imu9dof.hpp"

extern "C" {
#include "MahonyAHRS.h"
}
#include <cmath>

static const char *TAG = "Main";

// I2C配置参数
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_SCL_IO GPIO_NUM_4
#define I2C_MASTER_SDA_IO GPIO_NUM_5
#define I2C_MASTER_FREQ_HZ 400000

// 全局变量，LED Strip句柄
led_strip_handle_t led_strip;

// 定义四元数结构体
struct Quaternion {
    double w, x, y, z;
};

// 定义欧拉角结构体（弧度）
struct EulerAngles {
    float roll, pitch, yaw;
};



// 将弧度转换为角度
double radToDeg(double radians) {
    return radians * 180.0 / M_PI;
}

// 将角度转换为弧度
double degToRad(double deg) {
    return deg * M_PI / 180.0;
}

// 初始化I2C主机
static void i2c_master_init() {
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    conf.clk_flags = 0;

    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C参数配置失败: %d", err);
        return;
    }

    err = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C驱动安装失败: %d", err);
    }
}

// I2C扫描函数
void i2c_scanner() {
    ESP_LOGI(TAG, "Scanning I2C bus...\n");
    for (uint8_t addr = 1; addr < 127; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
        i2c_cmd_link_delete(cmd);

        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Device found at address 0x%02X\n", addr);
        }
    }
    ESP_LOGI(TAG, "Scan completed.\n");
}

/**
 * @brief 配置并初始化LED Strip
 * @return led_strip_handle_t LED Strip的句柄
 */
static led_strip_handle_t configure_led_strip(void)
{
    // LED Strip通用配置
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_STRIP_GPIO_NUM,    // 连接灯带数据线的GPIO
        .max_leds = LED_STRIP_LED_NUMBER,        // 灯带上最大LED数量
        .led_pixel_format = LED_PIXEL_FORMAT_GRB, // WS2812的像素格式通常是GRB
        .led_model = LED_MODEL_WS2812,           // LED模型
        // .flags.invert_out = false,               // 是否反转输出信号（如果使用电平转换器，有时需要）
    };

    // RMT后端特定配置
    led_strip_rmt_config_t rmt_config = {
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
        .clk_src = RMT_CLK_SRC_DEFAULT,          // 时钟源
        .resolution_hz = LED_STRIP_RMT_RES_HZ,   // RMT计数器时钟频率
        // .flags.with_dma = false,                 // 是否使用DMA（ESP32-S3支持）
#else
        .rmt_channel = 0,                        // 对于IDF v4.x，指定RMT通道
#endif
    };

    // LED Strip对象句柄
    led_strip_handle_t led_strip;
    // 创建新的RMT设备
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    ESP_LOGI(TAG, "Created LED strip object with RMT backend");
    return led_strip;
}

// qmc5883p_cal_data_t compensate_tilt_and_declination(qmc5883p_cal_data_t mag_data, MPU6050Data mpu_data, float declination_rad)
// {
//     qmc5883p_cal_data_t result;

//     // 1. 归一化加速度计数据
//     float norm = sqrtf(mpu_data.accel_x * mpu_data.accel_x +
//                        mpu_data.accel_y * mpu_data.accel_y +
//                        mpu_data.accel_z * mpu_data.accel_z);

//     if (norm < 0.001f) {
//         // 避免除以零，返回未处理的磁力计数据
//         return mag_data;
//     }

//     float ax_norm = mpu_data.accel_x / norm;
//     float ay_norm = mpu_data.accel_y / norm;
//     float az_norm = mpu_data.accel_z / norm;

//     // 2. 使用加速度计数据计算俯仰角(pitch)和横滚角(roll)
//     // 横滚角 (绕X轴旋转)
//     float roll = atan2f(ay_norm, az_norm);

//     // 俯仰角 (绕Y轴旋转)
//     float pitch = atan2f(-ax_norm, sqrtf(ay_norm * ay_norm + az_norm * az_norm));

//     // 3. 计算旋转矩阵的元素（从机体坐标系到地平坐标系）
//     float sin_roll = sinf(roll);
//     float cos_roll = cosf(roll);
//     float sin_pitch = sinf(pitch);
//     float cos_pitch = cosf(pitch);

//     // 4. 对磁力计数据进行倾斜补偿（从机体坐标系转换到地平坐标系）
//     // 水平分量X (北方向)
//     float hx = mag_data.x * cos_pitch +
//                mag_data.y * sin_roll * sin_pitch +
//                mag_data.z * cos_roll * sin_pitch;

//     // 水平分量Y (东方向)
//     float hy = mag_data.y * cos_roll -
//                mag_data.z * sin_roll;

//     // 垂直分量Z (地方向)
//     float hz = -mag_data.x * sin_pitch +
//                mag_data.y * sin_roll * cos_pitch +
//                mag_data.z * cos_roll * cos_pitch;

//     // 5. 对水平磁场分量进行磁偏角校正（从磁北到真北）
//     float north = hx * cosf(declination_rad) - hy * sinf(declination_rad);
//     float east = hx * sinf(declination_rad) + hy * cosf(declination_rad);
//     float down = hz; // 垂直分量不变

//     // 6. 返回结果
//     result.x = north;
//     result.y = east;
//     result.z = down;

//     return result;
// }

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "string.h"
#include "nvs_flash.h"
#include "nvs.h"

// 定义UART配置
#define UART_PORT_NUM      UART_NUM_1   // 使用的UART端口号
#define UART_BAUD_RATE     9600         // 串口设备的波特率
#define UART_RX_PIN        18           // ESP32的RX引脚（接设备的TX）
#define UART_TX_PIN        17           // ESP32的TX引脚（接设备的RX）
#define UART_RX_BUFFER_SIZE 1024        // RX缓冲区大小
#define UART_TX_BUFFER_SIZE 0           // TX缓冲区大小（若不发送可设为0）
#define UART_QUEUE_SIZE    10           // 事件队列大小
#define READ_BUF_SIZE      128          // 读取数据的缓冲区大小

void setup_uart() {
    // 步骤1: 配置UART参数
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,   // 8位数据位:cite[2]:cite[3]:cite[6]
        .parity = UART_PARITY_DISABLE,   // 无校验:cite[2]:cite[3]:cite[6]
        .stop_bits = UART_STOP_BITS_1,   // 1位停止位:cite[2]:cite[3]:cite[6]
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, // 无硬件流控:cite[3]
        .source_clk = UART_SCLK_DEFAULT, // 默认时钟源:cite[3]
    };
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));

    // 步骤2: 设置UART引脚
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // 步骤3: 安装UART驱动程序，设置缓冲区大小和事件队列
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, UART_RX_BUFFER_SIZE, UART_TX_BUFFER_SIZE, UART_QUEUE_SIZE, NULL, 0));
}

void read_and_print_uart_data() {
    uint8_t data[READ_BUF_SIZE]; // 数据缓冲区

    while (1) {
        // 检查数据长度:cite[2]:cite[6]
        int length = 0;
        ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_PORT_NUM, (size_t*)&length));
        if (length > 0) {
            // 读取数据，等待时间为100 ticks（可根据需要调整）:cite[2]:cite[6]
            int len = uart_read_bytes(UART_PORT_NUM, data, std::min(length, READ_BUF_SIZE), 100 / portTICK_PERIOD_MS);
            if (len > 0) {
                // 将接收到的数据打印为字符串（假设是文本数据）
                printf("Received: '%.*s'\n", len, (char*)data);
                // 如果需要打印十六进制值：
                // for (int i = 0; i < len; i++) {
                //     printf("0x%02X ", data[i]);
                // }
                // printf("\n");
            }
        }
        vTaskDelay(10 / portTICK_PERIOD_MS); // 稍作延迟，避免任务过于繁忙
    }
}

constexpr i2c_port_t mpu6050_i2c_num = I2C_NUM_0;
constexpr uint8_t mpu6050_addr = 0x68;
constexpr i2c_port_t qmc5883p_i2c_num = I2C_NUM_0;

// extern "C" void app_main()
// {
//     imu_sensor::IMU9DoF imu(mpu6050_i2c_num, mpu6050_addr, qmc5883p_i2c_num);
//     imu.init();
// }

// extern "C" void app_main(void)
// {
//     setup_uart();
//     printf("UART initialized, starting to read data...\n");
//     read_and_print_uart_data(); // 此函数包含无限循环，不会返回
// }

extern "C" void app_main()
{
    // 配置并初始化LED Strip
    led_strip = configure_led_strip();

    // 清除LED Strip（所有LED熄灭）
    ESP_ERROR_CHECK(led_strip_clear(led_strip));

    vTaskDelay(1000 / portTICK_PERIOD_MS); // 等待1秒

    // 初始化I2C
    i2c_master_init();
    ESP_LOGI(TAG, "I2C初始化完成");

    i2c_scanner();

    // 创建MPU6050对象并初始化
    MPU6050 mpu6050(I2C_MASTER_NUM, MPU6050_ADDR);

    if (esp_err_t ret = mpu6050.init(); ret != ESP_OK)
    {
        ESP_LOGE(TAG, "MPU6050初始化失败: err: 0x%x", ret);
        return;
    }
    ESP_LOGI(TAG, "MPU6050初始化成功");

    // 配置MPU6050使用辅助I2C
    if (esp_err_t mpu6050.enable_aux_i2c(); ret != ESP_OK) {
        ESP_LOGE(TAG, "启用MPU6050辅助I2C失败: err: 0x%x", ret);
        return;
    }
    ESP_LOGI(TAG, "MPU6050辅助I2C已启用");

    // 创建QMC5883P对象并初始化
    QMC5883P qmc5883p(I2C_MASTER_NUM);
    ret = qmc5883p.init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "QMC5883P初始化失败: %d", ret);
        return;
    }
    ESP_LOGI(TAG, "QMC5883P初始化成功");

    // mpu6050.calibrate_accel_gyro();

    // qmc5883p.calibrateMagnetometer();

    // 主循环读取传感器数据
    while (true) {
        int32_t start_time = esp_timer_get_time() / 1000;

        // 读取MPU6050数据
        mpu6050_data_t mpu_data, device_asix_mpu_data;
        ret = mpu6050.get_cal_motion(&mpu_data);
        if (ret == ESP_OK) {
            device_asix_mpu_data.accel_x = mpu_data.accel_x;
            device_asix_mpu_data.accel_y = -mpu_data.accel_y;
            device_asix_mpu_data.accel_z = -mpu_data.accel_z;
            device_asix_mpu_data.gyro_x = mpu_data.gyro_x;
            device_asix_mpu_data.gyro_y = -mpu_data.gyro_y;
            device_asix_mpu_data.gyro_z = -mpu_data.gyro_z;
            // ESP_LOGI(TAG, "加速度: X=%.2f, Y=%.2f, Z=%.2f",
            //         device_asix_mpu_data.accel_x, device_asix_mpu_data.accel_y, device_asix_mpu_data.accel_z);
            // ESP_LOGI(TAG, "陀螺仪: X=%.2f, Y=%.2f, Z=%.2f",
            //         device_asix_mpu_data.gyro_x, device_asix_mpu_data.gyro_y, device_asix_mpu_data.gyro_z);
            // ESP_LOGI(TAG, "温度: %.2f °C", mpu_data.temp);
        } else {
            ESP_LOGE(TAG, "读取MPU6050数据失败: %d", ret);
        }

        // 读取QMC5883P数据
        qmc5883p_cal_data_t mag_data, device_asix_mag_data;
        ret = qmc5883p.get_cal_data(&mag_data);
        if (ret == ESP_OK) {
            device_asix_mag_data.x = mag_data.y;
            device_asix_mag_data.y = mag_data.x;
            device_asix_mag_data.z = -mag_data.z;
            // ESP_LOGI(TAG, "磁力计: X=%.2f,\tY=%.2f,\tZ=%.2f",
            //         device_asix_mag_data.x, device_asix_mag_data.y, device_asix_mag_data.z);
        } else {
            ESP_LOGE(TAG, "读取QMC5883P数据失败: %d", ret);
        }

        device_asix_mag_data = compensate_tilt_and_declination(device_asix_mag_data, device_asix_mpu_data, -0.057f);
        // ESP_LOGI(TAG, "磁力计: X=%.2f,\tY=%.2f,\tZ=%.2f",
        //         mag_data.x, mag_data.y, mag_data.z);

        // MahonyAHRSupdateIMU(degToRad(mpu_data.gyro_x), degToRad(mpu_data.gyro_y), degToRad(mpu_data.gyro_z),
        //                     mpu_data.accel_x, mpu_data.accel_y, mpu_data.accel_z);

        MahonyAHRSupdate(degToRad(device_asix_mpu_data.gyro_x), degToRad(device_asix_mpu_data.gyro_y), degToRad(device_asix_mpu_data.gyro_z),
                         device_asix_mpu_data.accel_x, device_asix_mpu_data.accel_y, device_asix_mpu_data.accel_z,
                         device_asix_mag_data.x, device_asix_mag_data.y, device_asix_mag_data.z);

        Quaternion q;
        q.w = q0;
        q.x = q1;
        q.y = q2;
        q.z = q3;

        // 转换为欧拉角
        EulerAngles euler = quaternionToEuler(q);

        // ESP_LOGI(TAG, "欧拉角: Roll=%.2f°,\tPitch=%.2f°,\tYaw=%.2f°", radToDeg(euler.roll), radToDeg(euler.pitch), radToDeg(euler.yaw));

        // 示例1：设置第一个LED为红色
        ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, 0, 255, 0, 0)); // (句柄, LED索引, R, G, B)
        // 示例2：设置第二个LED为绿色
        ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, 1, 0, 255, 0));
        // 示例3：设置第三个LED为蓝色
        ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, 2, 0, 0, 255));

        // 将数据刷新到灯带
        ESP_ERROR_CHECK(led_strip_refresh(led_strip));

        int32_t end_time = esp_timer_get_time() / 1000;
        int32_t sleep_time = 10 - std::max((end_time - start_time), int32_t(0));
        vTaskDelay(pdMS_TO_TICKS(sleep_time)); // 每1秒读取一次
    }
}

extern "C" void app_main(void)
{
    // 1. 配置LED Strip
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_STRIP_GPIO_NUM,
        .max_leds = LED_STRIP_LED_NUMBER,
        .led_pixel_format = LED_PIXEL_FORMAT_GRB, // 注意颜色格式！
        .led_model = LED_MODEL_WS2812,
        .flags = {
            .invert_out = false,
        },
    };

    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = LED_STRIP_RMT_RES_HZ,
        .mem_block_symbols = 0,
        .flags = {
            .with_dma = false,
        },
    };

    // 2. 初始化LED Strip，并检查错误
    esp_err_t err = led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip);
    if (err != ESP_OK) {
        // 打印错误信息
        return;
    }

    CompressedLed compressed_led(led_strip);

    // 3. 清除可能存在的残留显示
    led_strip_clear(led_strip);
    vTaskDelay(pdMS_TO_TICKS(500));

    // 5. 务必刷新才能显示！
    led_strip_refresh(led_strip);

    float angle = 0.0f;
    while (1) {
        compressed_led.set_compressed_led_by_angle(angle);
        angle += 1.0f;

        led_strip_refresh(led_strip);

        vTaskDelay(pdMS_TO_TICKS(15));
    }
}