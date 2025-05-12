#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>

// Define the magic IOCTL number and commands.
#define BMP180_IOC_MAGIC 'b'
#define BMP180_IOC_SET_OSS _IOW(BMP180_IOC_MAGIC, 1, int)   // Lệnh IOCTL để đặt cài đặt oversampling (OSS)
#define BMP180_IOC_GET_OSS _IOR(BMP180_IOC_MAGIC, 2, int)   // Lệnh IOCTL để lấy cài đặt oversampling (OSS)
//#define BMP180_IOCTL_READ_TEMP_PRESSURE _IOR(BMP180_IOC_MAGIC, 1, int[2]) //changed the cmd

#define DEVICE_FILE "/dev/bmp180"   // Path to the character device file for BMP180

int main(int argc, char *argv[]) {
    int fd;                 // Bộ mô tả tập tin cho tập tin thiết bị BMP180
    int ret;                // Biến để lưu trữ các giá trị trả về từ các hàm
    char buffer[50];
    ssize_t bytes_read;
    int temperature_int, temperature_dec, pressure_int, pressure_dec;

    // Mở tập tin thiết bị BMP180 ở chế độ đọc-ghi
    fd = open(DEVICE_FILE, O_RDWR);
    if (fd < 0) {
        perror("Failed to open the device " DEVICE_FILE);
        return 1;
    }

    printf("Successfully opened %s.\n", DEVICE_FILE);

    // Kiểm tra và thực hiện các lệnh dựa trên đối số dòng lệnh
    if (argc > 1) {
        if (strcmp(argv[1], "set_oss") == 0) {  
            if (argc > 2) {                     
                int oss_to_set = atoi(argv[2]);    
                if (oss_to_set >= 0 && oss_to_set <= 3) { 
                    ret = ioctl(fd, BMP180_IOC_SET_OSS, &oss_to_set); 
                    if (ret == -1) {
                        perror("ioctl BMP180_IOC_SET_OSS failed"); 
                    } else {
                        printf("Set oversampling setting to %d.\n", oss_to_set); 
                    }
                } else {
                    fprintf(stderr, "Invalid oversampling setting. Must be between 0 and 3.\n"); 
                }
            } else {
                fprintf(stderr, "Usage: %s set_oss <0-3>\n", argv[0]); 
            }
        } else if (strcmp(argv[1], "get_oss") == 0) {
            int current_oss;
            ret = ioctl(fd, BMP180_IOC_GET_OSS, &current_oss);  
            if (ret == -1) { 
                perror("ioctl BMP180_IOC_GET_OSS failed");   
            } else {
                printf("Current oversampling setting: %d\n", current_oss);
            }
        } else if (strcmp(argv[1], "read") == 0 || argc == 1) { 
            bytes_read = read(fd, buffer, sizeof(buffer) - 1); 
            if (bytes_read > 0) {
                buffer[bytes_read] = '\0';
                // Phân tích chuỗi dữ liệu
                if (sscanf(buffer, "T:%d.%dC P:%d.%dhPa", &temperature_int, &temperature_dec, &pressure_int, &pressure_dec) == 4) {
                    printf("Temperature: %d.%d °C\n", temperature_int, temperature_dec);
                    printf("Pressure: %d.%02d hPa\n", pressure_int, pressure_dec);
                } else {
                    fprintf(stderr, "Error parsing read data: %s\n", buffer);
                }
            } else if (bytes_read == -1) {
                perror("Failed to read from " DEVICE_FILE);
            } else {
                printf("No data read.\n");
            }
        } else {
            fprintf(stderr, "Usage: %s [read | set_oss <0-3> | get_oss]\n", argv[0]);
        }
    } else {
        // Mặc định đọc dữ liệu nếu không có đối số
        bytes_read = read(fd, buffer, sizeof(buffer) - 1); 
        if (bytes_read > 0) {
            buffer[bytes_read] = '\0';
            // Phân tích chuỗi dữ liệu
            if (sscanf(buffer, "T:%d.%dC P:%d.%dhPa", &temperature_int, &temperature_dec, &pressure_int, &pressure_dec) == 4) {
                printf("Temperature: %d.%d °C\n", temperature_int, temperature_dec);
                printf("Pressure: %d.%02d hPa\n", pressure_int, pressure_dec);
            } else {
                fprintf(stderr, "Error parsing read data: %s\n", buffer);
            }
        } else if (bytes_read == -1) {
            perror("Failed to read from " DEVICE_FILE); 
        } else {
            printf("No data read.\n");
        }
    }

    close(fd);
    return 0;
}