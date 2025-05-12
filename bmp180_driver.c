#include <linux/init.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/kernel.h> // For KERN_INFO

#define DRIVER_NAME "bmp180_driver"// Tên của driver
#define DEVICE_NAME "bmp180" // Tên của device file sẽ được tạo (/dev/bmp180)
#define CLASS_NAME "bmp180"  // Tên của class device sẽ được tạo (/sys/class/bmp180)

#define BMP180_ADDR 0x77     // Địa chỉ I2C mặc định của BMP180

/* BMP180 registers */
#define BMP180_REG_CALIBRATION 0xAA   // Địa chỉ bắt đầu của dữ liệu hiệu chuẩn
#define BMP180_REG_CTRL_MEAS   0xF4   // Địa chỉ điều khiển đo lường
#define BMP180_REG_OUT_MSB     0xF6   // Địa chỉ bắt đầu của dữ liệu đo lường

/* BMP180 control values */
#define BMP180_CMD_TEMP        0x2E   // Lệnh đo nhiệt độ
#define BMP180_CMD_PRESSURE     0x34  // Lệnh đo áp suất
#define OSS 0    // Oversampling setting (0-3), 0 là độ phân giải thấp nhất, 3 là cao nhất
/* Calibration data structure */
struct bmp180_calibration {
    s16 AC1, AC2, AC3;   // Hệ số hiệu chuẩn liên quan đến nhiệt độ và áp suất
    u16 AC4, AC5, AC6;   // Hệ số hiệu chuẩn liên quan đến nhiệt độ và áp suất
    s16 B1, B2;          // Hệ số hiệu chuẩn liên quan đến áp suất
    s16 MB, MC, MD;      // Hệ số hiệu chuẩn liên quan đến nhiệt độ
};

struct bmp180_data {
    struct i2c_client *client;  // Con trỏ đến cấu trúc i2c_client đại diện cho thiết bị
    struct cdev cdev;           // Cấu trúc character device
    struct bmp180_calibration calib;  // Lưu trữ dữ liệu hiệu chuẩn
    dev_t devno;   // Số device (major và minor number)
    struct class *class;   // Con trỏ đến class device
    struct device *device;  // Con trỏ đến device trong /dev
};

static struct bmp180_data *bmp180_dev;   // Biến toàn cục trỏ đến cấu trúc dữ liệu của device BMP180
static int bmp180_major;   // Số major device được cấp phát

static int bmp180_read_calibration(struct i2c_client *client, struct bmp180_calibration *calib)
{
    u8 buf[22];   // Buffer để chứa 22 byte dữ liệu hiệu chuẩn
    int ret;

    ret = i2c_smbus_read_i2c_block_data(client, BMP180_REG_CALIBRATION, sizeof(buf), buf);
    if (ret < 0) {
        dev_err(&client->dev, "Failed to read calibration data: %d\n", ret);
        return ret;
    }
    //Giải mã dữ liệu hiệu chuẩn từ buffer
    calib->AC1 = (s16)((buf[0] << 8) | buf[1]);
    dev_info(&client->dev, "AC1: %d\n", calib->AC1);
    calib->AC2 = (s16)((buf[2] << 8) | buf[3]);
    dev_info(&client->dev, "AC2: %d\n", calib->AC2);
    calib->AC3 = (s16)((buf[4] << 8) | buf[5]);
    dev_info(&client->dev, "AC3: %d\n", calib->AC3);
    calib->AC4 = (u16)((buf[6] << 8) | buf[7]);
    dev_info(&client->dev, "AC4: %u\n", calib->AC4);
    calib->AC5 = (u16)((buf[8] << 8) | buf[9]);
    dev_info(&client->dev, "AC5: %u\n", calib->AC5);
    calib->AC6 = (u16)((buf[10] << 8) | buf[11]);
    dev_info(&client->dev, "AC6: %u\n", calib->AC6);
    calib->B1  = (s16)((buf[12] << 8) | buf[13]);
    dev_info(&client->dev, "B1: %d\n", calib->B1);
    calib->B2  = (s16)((buf[14] << 8) | buf[15]);
    dev_info(&client->dev, "B2: %d\n", calib->B2);
    calib->MB  = (s16)((buf[16] << 8) | buf[17]);
    dev_info(&client->dev, "MB: %d\n", calib->MB);
    calib->MC  = (s16)((buf[18] << 8) | buf[19]);
    dev_info(&client->dev, "MC: %d\n", calib->MC);
    calib->MD  = (s16)((buf[20] << 8) | buf[21]);
    dev_info(&client->dev, "MD: %d\n", calib->MD);

    return 0;
}
//Hàm đọc giá trị nhiệt độ thô từ cảm biến BMP180
static int bmp180_read_raw_temp(struct i2c_client *client, s32 *temp)
{
    int ret;
    u8 buf[2];

    // Gửi lệnh đo nhiệt độ
    ret = i2c_smbus_write_byte_data(client, BMP180_REG_CTRL_MEAS, BMP180_CMD_TEMP);
    if (ret < 0) {
        dev_err(&client->dev, "Failed to start temperature measurement: %d\n", ret);
        return ret;
    }

    msleep(5);

    // Đọc 2 byte kết quả nhiệt độ từ cảm biến 
    ret = i2c_smbus_read_i2c_block_data(client, BMP180_REG_OUT_MSB, 2, buf);
    if (ret < 0) {
        dev_err(&client->dev, "Failed to read temperature data: %d\n", ret);
        return ret;
    }

    // Kết hợp 2 byte để tạo thành giá trị nhiệt độ thô
    *temp = (buf[0] << 8) | buf[1];
    dev_dbg(&client->dev, "Raw temperature: %d\n", *temp);
    return 0;
}

//Hàm đọc giá trị áp suất thô từ cảm biến BMP180
static int bmp180_read_raw_pressure(struct i2c_client *client, s32 *pressure)
{
    int ret;
    u8 buf[3];

    // Gửi lệnh đo áp suất với độ phân giải OSS(Oversampling Setting)
    ret = i2c_smbus_write_byte_data(client, BMP180_REG_CTRL_MEAS, BMP180_CMD_PRESSURE + (OSS << 6));
    if (ret < 0) {
        dev_err(&client->dev, "Failed to start pressure measurement: %d\n", ret);
        return ret;
    }

    msleep(5 + (3 << OSS));  // Delay theo OSS

    // Đọc 3 byte kết quả áp suất từ cảm biến
    ret = i2c_smbus_read_i2c_block_data(client, BMP180_REG_OUT_MSB, 3, buf);
    if (ret < 0) {
        dev_err(&client->dev, "Failed to read pressure data: %d\n", ret);
        return ret;
    }

    // Kết hợp 3 byte để tạo thành giá trị áp suất thô
    *pressure = (((s32)buf[0] << 16) | ((s32)buf[1] << 8) | (s32)buf[2]) >> (8 - OSS);
    dev_dbg(&client->dev, "Raw pressure: %d\n", *pressure);
    return 0;
}

//Hàm tính toán nhiệt độ từ giá trị thô và dữ liệu hiệu chuẩn
static s32 bmp180_calculate_temp(s32 UT, struct bmp180_calibration *calib)
{
    s32 X1, X2, B5, T;

    X1 = (((s32)UT - (s32)calib->AC6) * (s32)calib->AC5) >> 15;
    X2 = ((s32)calib->MC << 11) / (X1 + (s32)calib->MD);
    B5 = X1 + X2;
    T = (B5 + 8) >> 4; 

    dev_dbg(&bmp180_dev->client->dev, "Calculated B5: %d, Temperature (x10): %d\n", B5, T);
    return T;
}

//Hàm tính toán áp suất thực tế từ giá trị áp suất thô, dữ liệu hiệu chuẩn và B5 (từ quá trình tính nhiệt độ).
static s32 bmp180_calculate_pressure(s32 UP, struct bmp180_calibration *calib, s32 B5)
{
    s32 B6, X1, X2, X3, B3, B7, p;
    u32 B4;

    B6 = B5 - 4000;
    X1 = (calib->B2 * (B6 * B6 >> 12)) >> 11;
    X2 = (calib->AC2 * B6) >> 11;
    X3 = X1 + X2;
    B3 = (((((s32)calib->AC1) * 4 + X3) << OSS) + 2) >> 2;

    X1 = (calib->AC3 * B6) >> 13;
    X2 = (calib->B1 * ((B6 * B6) >> 12)) >> 16;
    X3 = ((X1 + X2) + 2) >> 2;
    B4 = (calib->AC4 * (u32)(X3 + 32768)) >> 15;

    B7 = ((u32)UP - B3) * (50000 >> OSS);
    if (B7 < 0x80000000)
        p = (B7 << 1) / B4;
    else
        p = (B7 / B4) << 1;

    X1 = (p >> 8) * (p >> 8);
    X1 = (X1 * 3038) >> 16;
    X2 = (-7357 * p) >> 16;
    p += (X1 + X2 + 3791) >> 4; // Áp suất thực tế (Pa)

    dev_dbg(&bmp180_dev->client->dev, "Calculated B6: %d, X1: %d, X2: %d, X3: %d, B3: %d, B4: %u, B7: %u, Pressure: %d\n",
            B6, X1, X2, X3, B3, B4, B7, p);
    return p;
}

//Hàm đọc nhiệt độ và áp suất và lưu vào các biến con trỏ.
static int bmp180_read_temp_and_pressure(struct i2c_client *client, s32 *temperature, s32 *pressure)
{
    s32 UT, UP, B5, T, P;
    int ret;

    // Đọc nhiệt độ thô
    ret = bmp180_read_raw_temp(client, &UT);
    if (ret < 0) return ret;
    B5 = (((UT - bmp180_dev->calib.AC6) * bmp180_dev->calib.AC5) >> 15) +
         ((bmp180_dev->calib.MC << 11) / ((((UT - bmp180_dev->calib.AC6) * bmp180_dev->calib.AC5) >> 15) + bmp180_dev->calib.MD));
    T = (B5 + 8) >> 4;
    printk(KERN_INFO "Raw Temperature (UT): %d\n", UT);
    printk(KERN_INFO "Calculated Temperature (T): %d\n", T);

    // Đọc áp suất thô
    ret = bmp180_read_raw_pressure(client, &UP);
    if (ret < 0) return ret;
    P = bmp180_calculate_pressure(UP, &bmp180_dev->calib, B5);
    printk(KERN_INFO "Raw Pressure (UP): %d\n", UP);
    printk(KERN_INFO "Calculated Pressure (P): %d\n", P);

    // Gán giá trị nhiệt độ và áp suất cho các con trỏ
    *temperature = T;
    *pressure = P;
    dev_dbg(&client->dev, "Temperature (x10): %d, Pressure: %d\n", *temperature, *pressure);
    return 0;
}

//Hàm open của device file. Được gọi khi userspace mở device file.
static int bmp180_open(struct inode *inode, struct file *filp)
{
    filp->private_data = bmp180_dev;
    dev_info(&bmp180_dev->client->dev, "Device opened\n");
    return 0;
}

//Hàm release của device file. Được gọi khi userspace đóng device file.
static int bmp180_release(struct inode *inode, struct file *filp)
{
    dev_info(&bmp180_dev->client->dev, "Device released\n");
    return 0;
}

//Hàm read device file. Được gọi khi userspace đọc dữ liệu từ device file.
static ssize_t bmp180_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
    struct bmp180_data *dev = filp->private_data;
    s32 temperature, pressure;
    char result[40];
    int ret;
    ssize_t len;

    ret = bmp180_read_temp_and_pressure(dev->client, &temperature, &pressure);
    if (ret < 0)
        return ret;

    // Convert: nhiệt độ (°C x10), áp suất (Pa)
    printk(KERN_INFO "Temperature (x10): %d, Pressure: %d\n", temperature, pressure);
    len = snprintf(result, sizeof(result), "T:%d.%dC P:%d.%02dhPa\n",
                   temperature / 10, abs(temperature % 10),
                   pressure / 100, pressure % 100);

    if (copy_to_user(buf, result, len))
        return -EFAULT;

    return len;
}

static const struct file_operations bmp180_fops = {
    .owner = THIS_MODULE,
    .open = bmp180_open,
    .release = bmp180_release,
    .read = bmp180_read,
};

static int bmp180_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int ret;
    struct device *dev;

    dev_info(&client->dev, "bmp180_probe called\n"); // In ra thông báo khi probe được gọi

    // Cấp phát bộ nhớ cho cấu trúc dữ liệu của device
    bmp180_dev = devm_kzalloc(&client->dev, sizeof(*bmp180_dev), GFP_KERNEL);
    if (!bmp180_dev)
        return -ENOMEM;

    bmp180_dev->client = client;

    // Đọc dữ liệu hiệu chuẩn từ BMP180
    ret = bmp180_read_calibration(client, &bmp180_dev->calib);
    if (ret < 0) {
        dev_err(&client->dev, "Failed to read calibration data\n");
        return ret;
    }
    dev_info(&client->dev, "Calibration data read successfully\n");

    // Cấp phát vùng số device (major và minor number)
    ret = alloc_chrdev_region(&bmp180_dev->devno, 0, 1, DEVICE_NAME);
    if (ret < 0) {
        dev_err(&client->dev, "Failed to allocate char device region\n");
        goto err_free;
    }
    bmp180_major = MAJOR(bmp180_dev->devno);
    dev_info(&client->dev, "Allocated char device region: major=%d\n", MAJOR(bmp180_dev->devno));

    // Khởi tạo cấu trúc cdev (character device)
    cdev_init(&bmp180_dev->cdev, &bmp180_fops);
    bmp180_dev->cdev.owner = THIS_MODULE;

    // Thêm cdev vào hệ thống
    ret = cdev_add(&bmp180_dev->cdev, bmp180_dev->devno, 1);
    if (ret < 0) {
        dev_err(&client->dev, "Failed to add char device\n");
        goto err_unreg;
    }
    dev_info(&client->dev, "Char device added\n");

    // Tạo class device
    bmp180_dev->class = class_create(CLASS_NAME);
    if (IS_ERR(bmp180_dev->class)) {
        ret = PTR_ERR(bmp180_dev->class);
        dev_err(&client->dev, "Failed to create device class\n");
        goto err_cdev;
    }
    dev_info(&client->dev, "Device class created\n");

    // Tạo device node trong /dev
    dev = device_create(bmp180_dev->class, NULL, bmp180_dev->devno, bmp180_dev, DEVICE_NAME);
    if (IS_ERR(dev)) {
        ret = PTR_ERR(dev);
        dev_err(&client->dev, "Failed to create device\n");
        goto err_class;
    }
    bmp180_dev->device = dev; // Lưu trữ con trỏ device
    dev_info(&client->dev, "Device created\n");

    i2c_set_clientdata(client, bmp180_dev);

    dev_info(&client->dev, "BMP180 driver initialized\n"); // In ra log khi driver được khởi tạo thành công
    return 0;

err_class:
    class_destroy(bmp180_dev->class); // Hủy class
err_cdev:
    cdev_del(&bmp180_dev->cdev); // Xóa cdev
err_unreg:
    unregister_chrdev_region(bmp180_dev->devno, 1); // Hủy đăng ký vùng số device
err_free:
    return ret;
}

//Hàm xóa driver khi không còn sử dụng
static void bmp180_remove(struct i2c_client *client)
{
    struct bmp180_data *dev = i2c_get_clientdata(client);
    device_destroy(dev->class, dev->devno);
    class_destroy(dev->class);
    cdev_del(&dev->cdev);
    unregister_chrdev_region(dev->devno, 1);
    dev_info(&client->dev, "BMP180 driver removed\n");
}

// Định nghĩa ID của thiết bị để kernel có thể nhận diện driver này
static const struct i2c_device_id bmp180_id[] = {
    { .name = "bmp180", .driver_data = 0 },
    { .name = "bme180", .driver_data = 0 }, // Thử thêm một ID tương tự
    { .name = "bosch,bmp180", .driver_data = 0 }, // Thử ID có tiền tố nhà sản xuất
    { }
};
MODULE_DEVICE_TABLE(i2c, bmp180_id);

//Cấu trúc i2c_driver để đăng ký driver BMP180 với hệ thống I2C.
static struct i2c_driver bmp180_driver = {
    .driver = {
        .name   = DRIVER_NAME,
        .owner  = THIS_MODULE,
    },
    .probe      = bmp180_probe,
    .remove     = bmp180_remove,
    .id_table   = bmp180_id,
};

// Đăng ký driver I2C với kernel
module_i2c_driver(bmp180_driver);

MODULE_AUTHOR("Your Name");
MODULE_DESCRIPTION("BMP180 I2C Driver");
MODULE_LICENSE("GPL");